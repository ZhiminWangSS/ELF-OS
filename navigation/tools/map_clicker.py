#!/usr/bin/env python3
"""Web map click-to-navigate for supervised short tests.

Serves the prior map on http://<robot>:8018. A click-drag picks (x, y, yaw);
the server validates localization health, goal free-space clearance and the
short-test distance cap, then runs tools/hardware_test.py --execute with the
picked goal (same supervised flow as the shell workflow). No auth: only bind
to interfaces you trust (Tailscale)."""
import json, math, os, subprocess, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ROOT = Path(__file__).resolve().parents[1]
MAPDIR = ROOT / 'maps/floor_1789552084236'
ROBOT = yaml.safe_load((ROOT / 'config/robot.yaml').read_text())
CLEAR_RADIUS = max(np.linalg.norm(v) for v in ROBOT['footprint']) + .05
MAX_GOAL_DIST = 15
PPM = 40  # rendered pixels per metre

SKILL_PATH = ROOT.parent / '.zcode' / 'skills' / 'elf-nav' / 'SKILL.md'
PLACES_PATH = ROOT / 'config' / 'places.yaml'
LLM_CFG_PATH = ROOT / 'config' / 'llm.local.yaml'  # gitignored: {base_url, api_key, model}
PLACES = {}
LLM = {'base_url': '', 'api_key': '', 'model': ''}

cfg = yaml.safe_load((MAPDIR / 'map.yaml').read_text())
grid = np.flipud(np.array(Image.open(MAPDIR / cfg['image'])))
clearance = distance_transform_edt(grid == 254) * cfg['resolution']
X0, Y0 = cfg['origin'][0], cfg['origin'][1]
RES = cfg['resolution']
W_M, H_M = grid.shape[1] * RES, grid.shape[0] * RES


def cell_clearance(wx, wy):
    cx, cy = int((wx - X0) / RES), int((wy - Y0) / RES)
    if not (0 <= cx < grid.shape[1] and 0 <= cy < grid.shape[0]):
        return -1.0
    return float(clearance[cy, cx])


def load_places():
    global PLACES
    if PLACES_PATH.exists():
        try:
            PLACES = yaml.safe_load(PLACES_PATH.read_text()) or {}
        except Exception:
            PLACES = {}


def save_places():
    tmp = PLACES_PATH.with_suffix('.tmp')
    tmp.write_text(yaml.safe_dump(PLACES, allow_unicode=True, sort_keys=True))
    os.replace(tmp, PLACES_PATH)


def place_save(name):
    """Record the robot's CURRENT pose under a name (Method A annotation)."""
    name = (name or '').strip()
    if not name:
        return {'ok': False, 'error': '地点名不能为空'}
    s = locator.snapshot()
    if not s['healthy'] or not s['fresh'] or not s['pose']:
        return {'ok': False, 'error': '定位不健康，无法记录位置'}
    if name in PLACES:
        return {'ok': False, 'error': f'「{name}」已存在（{PLACES[name]["x"]:.2f},{PLACES[name]["y"]:.2f}），换个名字或先删除'}
    PLACES[name] = {'x': round(s['pose'][0], 3), 'y': round(s['pose'][1], 3),
                    'yaw': round(s['pose'][2], 1), 'aliases': [], 'note': 'web-marked'}
    save_places()
    return {'ok': True, 'name': name, 'pose': PLACES[name]}


_STRIP_WORDS = ['走到', '去', '导航到', '房间', '门口', '门', '号', '室', '的', 'room', '号房']


def _norm(text):
    t = str(text or '').strip().lower()
    for w in _STRIP_WORDS:
        t = t.replace(w, '')
    return t.replace(' ', '')


def place_resolve(query):
    """Query -> pose. Exact name/alias, then substring both ways."""
    q = _norm(query)
    if not q:
        return {'ok': False, 'error': '请输入地点名', 'places': list(PLACES)}
    for name, item in PLACES.items():
        if _norm(name) == q or any(_norm(a) == q for a in item.get('aliases', [])):
            return {'ok': True, 'name': name, 'x': item['x'], 'y': item['y'], 'yaw_deg': item['yaw']}
    cands = [name for name in PLACES if q in _norm(name)
             or any(q in _norm(a) for a in PLACES[name].get('aliases', []))]
    if len(cands) == 1:
        name = cands[0]
        item = PLACES[name]
        return {'ok': True, 'name': name, 'x': item['x'], 'y': item['y'], 'yaw_deg': item['yaw']}
    if cands:
        return {'ok': False, 'error': '匹配到多个地点，请点选：', 'candidates': cands}
    return {'ok': False, 'error': f'没有叫「{query}」的地点', 'places': list(PLACES)}


def load_llm_cfg():
    """OpenAI-compatible chat endpoint. llm.local.yaml < env vars < zcode
    desktop install's coding-plan key (read from ~/.zcode, never committed)."""
    if LLM_CFG_PATH.exists():
        try:
            LLM.update({k: str(v) for k, v in (yaml.safe_load(LLM_CFG_PATH.read_text()) or {}).items()
                        if k in LLM})
        except Exception:
            pass
    for k in LLM:
        LLM[k] = os.environ.get('ELF_LLM_' + k.upper(), '') or LLM[k]
    if not (LLM['base_url'] and LLM['api_key'] and LLM['model']):
        try:
            pc = json.loads(Path.home().joinpath('.zcode/v2/provider_config.json').read_text())
            key = pc['config']['providerConfigRules']['providerRules'][0]['config']['access']['apiKey']
            LLM['base_url'] = LLM['base_url'] or 'https://api.z.ai/api/coding/paas/v4'
            LLM['api_key'] = LLM['api_key'] or key
            LLM['model'] = LLM['model'] or 'GLM-5.3-Flash'
        except Exception:
            pass
    LLM['base_url'] = LLM['base_url'].rstrip('/')


def _skill_prompt():
    """Body of the elf-nav zcode skill: the system prompt for /ask chats."""
    try:
        return SKILL_PATH.read_text().split('---', 2)[2].strip()
    except Exception:
        return ''


_FALLBACK_ASK_SYSTEM = (
    '你是机器狗的导航指令解析器。把用户指令解析为一个目标点，只输出一个 JSON 对象：\n'
    '{"name":"地点名或说明","x":数字,"y":数字,"yaw_deg":数字,"reply":"一句话中文说明"}。\n'
    '无法确定目标时输出 {"error":"原因"}。优先匹配地点表；坐标必须在地图范围内。')


def llm_chat(system, user):
    """One round through <base_url>/chat/completions. Returns content text."""
    body = json.dumps({'model': LLM['model'], 'temperature': 0, 'max_tokens': 900, 'messages': [
        {'role': 'system', 'content': system}, {'role': 'user', 'content': user}]}).encode()
    req = urllib.request.Request(LLM['base_url'] + '/chat/completions', data=body, headers={
        'Content-Type': 'application/json', 'Authorization': 'Bearer ' + LLM['api_key']})
    with urllib.request.urlopen(req, timeout=25) as r:
        data = json.loads(r.read())
    return data['choices'][0]['message']['content']


def _clip_goal(x, y, yaw):
    x = min(max(float(x), X0 + 0.1), X0 + W_M - 0.1)
    y = min(max(float(y), Y0 + 0.1), Y0 + H_M - 0.1)
    return x, y, ((float(yaw) + 180) % 360) - 180 if yaw is not None else 0.0


def ask_goal(text):
    """Natural-language instruction -> goal pose. LLM when configured,
    else rule-based place matching."""
    text = (text or '').strip()
    if not text:
        return {'ok': False, 'error': '请输入指令'}
    llm_ready = bool(LLM['base_url'] and LLM['api_key'] and LLM['model'])
    if llm_ready:
        table = '\n'.join(f"{n}: x={v['x']}, y={v['y']}, yaw={v['yaw']}"
                          + (f", aliases={v['aliases']}" if v.get('aliases') else '')
                          for n, v in PLACES.items()) or '(空)'
        s = locator.snapshot()
        pose_line = (f"机器狗当前位姿：x={s['pose'][0]:.2f}, y={s['pose'][1]:.2f}, yaw={s['pose'][2]:.1f}°\n"
                     if s['pose'] else '')
        dyn = ('\n\n## 动态上下文\n地点表（名称: x, y, yaw 度）：\n' + table + '\n' + pose_line +
               f'地图范围 x∈[{X0:.1f},{X0 + W_M:.1f}], y∈[{Y0:.1f},{Y0 + H_M:.1f}]。\n'
               '用户指令见下一条消息，按契约只输出一个 JSON 对象。')
        system = (_skill_prompt() or _FALLBACK_ASK_SYSTEM) + dyn
        try:
            raw = llm_chat(system, text)
            js = raw[raw.find('{'):raw.rfind('}') + 1]
            d = json.loads(js)
            if 'error' in d and d['error']:
                return {'ok': False, 'error': str(d['error'])[:200]}
            x, y, yaw = _clip_goal(d['x'], d['y'], d.get('yaw_deg', 0))
            return {'ok': True, 'source': 'llm', 'name': str(d.get('name', ''))[:40],
                    'x': x, 'y': y, 'yaw_deg': yaw, 'reply': str(d.get('reply', ''))[:120]}
        except Exception as exc:
            fallback_note = f'（大模型调用失败：{str(exc)[:80]}，退回地点名匹配）'
        else:
            fallback_note = ''
    else:
        fallback_note = ''
    r = place_resolve(text)
    if r.pop('places', None) is not None:
        pass  # hint list handled client-side from /places
    if r['ok']:
        r['source'] = 'rule'
    elif not llm_ready:
        r['error'] = str(r.get('error', '')) + '（未配置大模型 API，仅支持直接输入地点名；配置见 navigation/config/llm.local.yaml）'
    elif fallback_note:
        r['error'] = str(r.get('error', '')) + fallback_note
    return r


try:
    FONT = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 13)
except Exception:
    FONT = None


def render_map_png():
    # Dark "blueprint" palette: deep slate free space, bright walls, subtle grid.
    lut = np.full((256, 3), (12, 17, 24), dtype=np.uint8)   # unknown -> darkest
    lut[254] = (37, 53, 72)    # free -> slate, clearly lighter than unknown
    lut[0] = (223, 233, 242)   # occupied -> bright walls
    img = Image.fromarray(lut[grid[::-1]]).convert('RGB')
    img = img.resize((int(W_M * PPM), int(H_M * PPM)), Image.LANCZOS)
    d = ImageDraw.Draw(img, 'RGBA')
    for meter in range(0, int(max(W_M, H_M)) + 1, 2):
        px = int((meter - X0) * PPM) if X0 <= meter <= X0 + W_M else -1
        py = int((Y0 + H_M - meter) * PPM) if Y0 <= meter <= Y0 + H_M else -1
        major = meter % 10 == 0
        if px >= 0:
            d.line([(px, 0), (px, img.height)], fill=(56, 189, 248, 60 if major else 22), width=2 if major else 1)
        if py >= 0:
            d.line([(0, py), (img.width, py)], fill=(56, 189, 248, 60 if major else 22), width=2 if major else 1)
        if major:
            if px >= 0:
                d.text((px + 5, 7), str(meter), font=FONT, fill=(125, 211, 252, 220))
            if py >= 0:
                d.text((7, py + 5), str(meter), font=FONT, fill=(125, 211, 252, 220))
    buf = __import__('io').BytesIO()
    img.save(buf, 'PNG')
    return buf.getvalue()


class Locator(Node):
    def __init__(self):
        super().__init__('map_clicker_locator')
        self.lock = threading.Lock()
        self.pose, self.healthy, self.stamp = None, False, 0.
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(Bool, '/navigation/localization_healthy', self._health, 10)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.pose = (p.x, p.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y))))
            self.stamp = time.monotonic()

    def _health(self, m):
        with self.lock:
            self.healthy = bool(m.data)

    def snapshot(self):
        with self.lock:
            return {'pose': self.pose, 'healthy': self.healthy, 'fresh': time.monotonic() - self.stamp < 1.0}


locator = None
busy = {'active': False, 'since': 0, 'log': ''}


def clear_costmaps_around_robot():
    """Best-effort: drop stale obstacle-layer marks near the robot before a goal.
    Foxy nav2 exposes these as <costmap>/clear_around_<costmap> with an empty
    ClearCostmapAroundRobot request. Real obstacles are re-marked by the 10 Hz
    live cloud within one update."""
    for svc in ('/local_costmap/clear_around_local_costmap',
                '/global_costmap/clear_around_global_costmap'):
        try:
            subprocess.run(['/usr/bin/timeout', '4', '/opt/ros/foxy/bin/ros2', 'service', 'call',
                            svc, 'nav2_msgs/srv/ClearCostmapAroundRobot'],
                           capture_output=True, env=os.environ.copy(), timeout=6)
        except Exception:
            pass


def _run_session(x, y, yaw_deg, d):
    """One supervised hardware_test session to a single goal. Returns (ok, detail)."""
    # Include an in-place rotation allowance: stumpy health-gated rotation of
    # up to 180 deg can take 25+ s in this corridor before the walk begins.
    goal_timeout = min(140, round(30 + d / 0.2 + 12))
    seconds = min(175, goal_timeout + 12)
    clear_costmaps_around_robot()
    cmd = ['/usr/bin/python3', str(ROOT / 'tools/hardware_test.py'), '--execute',
           '--seconds', str(seconds), '--goal-x', f'{x:.3f}', '--goal-y', f'{y:.3f}',
           '--goal-yaw-deg', f'{yaw_deg:.1f}', '--max-distance', f'{min(15, d + 0.5):.2f}',
           '--goal-timeout', str(goal_timeout)]
    # rclpy/Nav2 live behind env.bash: the server process was started from a
    # sourced shell, so pass its environment through instead of a clean one.
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=seconds + 100,
                       cwd=str(ROOT), env=os.environ.copy())
    out = (r.stdout + r.stderr).strip().splitlines()
    ok = r.returncode == 0 and '"status": 4' in r.stdout
    if ok:
        return True, ''
    if '"status": 6' in r.stdout:
        detail = ('Nav2 中止：路上或起步处的代价地图被障碍/旧标记挡住（行为树已自动清图重试 3 次仍未通过）。'
                  '可稍候重试同一目标；若目标方向的通道确实被实物堵死，请换一个目标或用遥控器挪开障碍')
    else:
        detail = next((l for l in reversed(out) if l.strip()), 'unknown')
    return False, detail[:300]


def plan_preview(x, y, yaw_deg):
    """Plan-only preview via Nav2 ComputePathToPose: returns the route the
    robot would take. Sends no motion command of any kind."""
    if busy['active']:
        return {'ok': False, 'error': '导航进行中，无法预览'}
    s = locator.snapshot()
    if not s['healthy'] or not s['fresh'] or not s['pose']:
        return {'ok': False, 'error': '定位不健康，无法规划'}
    d = math.hypot(x - s['pose'][0], y - s['pose'][1])
    if d > MAX_GOAL_DIST:
        return {'ok': False, 'error': f'目标距离 {d:.1f} m 超过单段上限 {MAX_GOAL_DIST} m'}
    c = cell_clearance(x, y)
    if c < CLEAR_RADIUS:
        return {'ok': False, 'error': f'目标净空 {c:.2f} m 不足（需 {CLEAR_RADIUS:.2f} m）'}
    if not locator.plan_client.wait_for_server(timeout_sec=3):
        return {'ok': False, 'error': 'Nav2 规划器不可用（导航栈未就绪？）'}
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = locator.get_clock().now().to_msg()
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    goal = ComputePathToPose.Goal()
    goal.pose = pose
    goal.planner_id = 'GridBased'
    future = locator.plan_client.send_goal_async(goal)
    end = time.monotonic() + 6
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not future.done():
        return {'ok': False, 'error': '规划请求超时'}
    handle = future.result()
    if not handle.accepted:
        return {'ok': False, 'error': '规划器拒绝该目标'}
    rf = handle.get_result_async()
    end = time.monotonic() + 8
    while not rf.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not rf.done():
        return {'ok': False, 'error': '规划超时'}
    result = rf.result()
    if result.status != 4 or len(result.result.path.poses) < 2:
        return {'ok': False, 'error': '无可行路径（目标或必经区域被占据/未知）'}
    pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in result.result.path.poses]
    plen = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    return {'ok': True, 'start': [round(s['pose'][0], 2), round(s['pose'][1], 2), round(s['pose'][2], 1)],
            'goal': [round(x, 2), round(y, 2), round(((yaw_deg + 180) % 360) - 180, 1)],
            'distance_m': round(d, 2), 'path_len_m': round(plen, 2),
            'est_s': round(plen / 0.2 + 15), 'path': pts}


def cancel_nav():
    """Sanctioned stop path: goal.py --cancel (Nav2 cancel + motion lock)."""
    try:
        subprocess.run(['/usr/bin/python3', str(ROOT / 'tools/goal.py'), '--cancel'],
                       capture_output=True, timeout=15, env=os.environ.copy())
        return {'ok': True, 'error': '已请求停止，会话将自动收尾'}
    except Exception as exc:
        return {'ok': False, 'error': str(exc)[:200]}


def run_goal(x, y, yaw_deg):
    """One supervised goal.py session: checks, walk, final rotation - all via
    the repository's standard tool. No staging, no custom rotation."""
    here = locator.snapshot()
    d = math.hypot(x - here['pose'][0], y - here['pose'][1]) if here['pose'] else None
    if not here['healthy'] or not here['fresh']:
        return {'ok': False, 'error': '定位不健康，拒绝发目标'}
    if d is None or d > MAX_GOAL_DIST:
        return {'ok': False, 'error': f'目标距离 {d and round(d,1)} m 超过单段上限 {MAX_GOAL_DIST} m'}
    c = cell_clearance(x, y)
    if c < CLEAR_RADIUS:
        return {'ok': False, 'error': f'目标净空 {c:.2f} m 不足（需 {CLEAR_RADIUS:.2f} m）'}
    busy.update(active=True, since=time.monotonic(), log='导航中…')
    t0 = time.monotonic()
    try:
        ok, detail = _run_session(x, y, yaw_deg, d)
        busy.update(active=False, log='')
        return {'ok': ok, 'error': '' if ok else detail,
                'distance_m': round(d, 2), 'duration_s': round(time.monotonic() - t0, 1)}
    except subprocess.TimeoutExpired:
        busy.update(active=False, log='')
        return {'ok': False, 'error': '会话超时，接口已按上限关闭',
                'distance_m': round(d, 2), 'duration_s': round(time.monotonic() - t0, 1)}


PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ELF 地图选点导航</title>
<style>
:root{--bg:#0b0f14;--panel:#111926;--line:#1e2a38;--text:#e6edf3;--muted:#8b98a5;
--accent:#38bdf8;--ok:#34d399;--warn:#fbbf24;--err:#f87171;--goal:#fb7185}
*{box-sizing:border-box}
html,body{height:100%}
body{font:14px/1.5 -apple-system,"PingFang SC","Microsoft YaHei","Segoe UI",system-ui,sans-serif;
background:radial-gradient(1100px 700px at 75% -10%,#12202e 0%,var(--bg) 55%);
color:var(--text);margin:0;display:flex;flex-direction:column;height:100vh;overflow:hidden}
header{display:flex;align-items:center;gap:10px;padding:10px 16px;background:rgba(17,25,38,.9);
border-bottom:1px solid var(--line);flex:none;flex-wrap:wrap}
.logo{width:28px;height:28px;border-radius:9px;background:linear-gradient(135deg,#38bdf8,#1d4ed8);
display:flex;align-items:center;justify-content:center;font-size:15px;flex:none;
color:#fff;font-weight:800;font-family:inherit}
h1{font-size:15px;margin:0;font-weight:650;letter-spacing:.3px;white-space:nowrap}
h1 small{color:var(--muted);font-weight:400;margin-left:8px;font-size:11.5px}
.chip{display:inline-flex;align-items:center;gap:7px;padding:4px 12px;border-radius:999px;
background:#0c141d;border:1px solid var(--line);font-size:12.5px;color:var(--muted);
font-variant-numeric:tabular-nums;white-space:nowrap}
.chip .dot{width:7px;height:7px;border-radius:50%;background:#5b6b7a;flex:none}
.chip.ok{color:#a7f3d0;border-color:rgba(52,211,153,.35)}
.chip.ok .dot{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.chip.warn{color:#fecaca;border-color:rgba(248,113,113,.4)}
.chip.warn .dot{background:var(--err);box-shadow:0 0 8px var(--err)}
#h-phase{margin-left:auto;color:#bfdbfe;border-color:rgba(56,189,248,.35)}
#nlbar{display:flex;gap:8px;padding:10px 16px;background:rgba(13,21,32,.92);
border-bottom:1px solid var(--line);flex:none;align-items:center;flex-wrap:wrap}
#cmdin{flex:1;min-width:220px;max-width:560px;padding:9px 14px;border-radius:10px;
border:1px solid var(--line);background:#0a111a;color:var(--text);font-size:14px;font-family:inherit}
#cmdin:focus{outline:none;border-color:var(--accent);box-shadow:0 0 0 3px rgba(56,189,248,.15)}
#nlbar .go{flex:none;padding:9px 18px;font-size:14px}
#nlbar .mark{flex:none;padding:9px 14px;font-size:13px}
#main{display:flex;flex:1;min-height:0}
#mapwrap{flex:1;position:relative;overflow:hidden;background:#070b10}
#map{position:absolute;left:0;top:0;transform-origin:top left}
#cv{position:absolute;left:0;top:0;cursor:crosshair}
#legend{position:absolute;left:12px;bottom:12px;display:flex;gap:8px;pointer-events:none;flex-wrap:wrap}
#legend span{display:inline-flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;
background:rgba(10,15,22,.78);border:1px solid var(--line);font-size:12px;color:var(--muted);backdrop-filter:blur(4px)}
#legend i{width:9px;height:9px;border-radius:3px;flex:none}
aside{width:330px;flex:none;background:var(--panel);border-left:1px solid var(--line);
padding:14px;display:flex;flex-direction:column;gap:12px;overflow-y:auto}
aside::-webkit-scrollbar{width:8px}
aside::-webkit-scrollbar-thumb{background:#223042;border-radius:4px}
.card{background:#0d1520;border:1px solid var(--line);border-radius:12px;padding:14px}
.card-title{font-size:12px;font-weight:650;color:var(--accent);letter-spacing:1px;margin-bottom:10px}
.row{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px dashed #1a2532;font-size:13px}
.row:last-of-type{border-bottom:none}
.row span{color:var(--muted)}
.row b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
#result{font-size:13px;line-height:1.7;white-space:pre-wrap}
#result .ok{color:#a7f3d0}#result .err{color:#fca5a5}
#prog{display:none;font-size:13.5px;color:#fde68a;align-items:center;gap:8px}
#prog::before{content:"";width:12px;height:12px;border:2px solid rgba(251,191,36,.35);
border-top-color:#fbbf24;border-radius:50%;animation:spin .9s linear infinite;flex:none}
@keyframes spin{to{transform:rotate(360deg)}}
#hint{font-size:13px;color:var(--muted);line-height:1.9}
#hint b{color:#cfe3f5}
.actions{display:flex;gap:8px;margin-top:14px}
button{border:none;border-radius:10px;font-size:14.5px;cursor:pointer;padding:11px 14px;
transition:transform .06s,filter .15s;font-family:inherit}
button:active{transform:scale(.97)}
button:disabled{opacity:.4;cursor:default}
.primary{flex:2;background:linear-gradient(135deg,#0d9f6e,#057a55);color:#ecfdf5;font-weight:600;
box-shadow:0 4px 14px rgba(13,159,110,.25)}
.primary:hover{filter:brightness(1.12)}
.ghost{flex:1;background:#16202e;color:#aebccd;border:1px solid var(--line)}
.ghost:hover{background:#1b2735}
.danger{width:100%;background:linear-gradient(135deg,#dc2626,#991b1b);color:#fee2e2;font-weight:600;
box-shadow:0 4px 14px rgba(220,38,38,.25)}
.danger:hover{filter:brightness(1.12)}
</style></head><body>
<header>
 <div class="logo">E</div>
 <h1>ELF 地图选点导航<small>SUPERVISED NAV</small></h1>
 <span class="chip" id="h-pose"><span class="dot"></span><span class="t">位置获取中…</span></span>
 <span class="chip" id="h-health"><span class="dot"></span><span class="t">…</span></span>
 <span class="chip" id="h-phase">空闲</span>
</header>
<div id="nlbar">
 <input id="cmdin" list="placelist" placeholder='输入指令，例如"去1705门口"或"带我去有打印机的那间"'>
 <datalist id="placelist"></datalist>
 <button class="primary go" onclick="sendAsk()">🧠 解析目标</button>
 <button class="ghost mark" title="把机器狗当前位置保存为一个地点" onclick="markPlace()">📍 标记当前位置</button>
</div>
<div id="main">
 <div id="mapwrap">
  <img id="map" alt="map"><canvas id="cv"></canvas>
  <div id="legend">
   <span><i style="background:#38bdf8"></i>机器人</span>
   <span><i style="background:#fb7185"></i>目标点</span>
   <span><i style="background:#34d399"></i>规划路径</span>
   <span><i style="background:#f59e0b"></i>地点</span>
  </div>
 </div>
 <aside>
  <div class="card" id="panel">
   <div class="card-title">导航任务</div>
   <div id="hint">在地图上<b>按住拖动</b>选择目标点与到达朝向：
    起点=目标位置，拖动方向=到达后的朝向。<br><br>
    松开后自动规划路线并预览，<b>按下「开始导航」才会真正出发</b>。<br><br>
    单段上限 15 m ｜ 遥控器随时接管 ｜ 急停可用</div>
  </div>
  <div class="card"><div class="card-title">执行状态</div><div id="result"></div><div id="prog"></div></div>
 </aside>
</div>
<script>
const PPM=__PPM__, X0=__X0__, Y1=__Y1__, S=__SCALE__;
const $=id=>document.getElementById(id);
const img=$("map"), cv=$("cv"), ctx=cv.getContext("2d");
let pose=null, phase="idle", sel=null, drag=null, plan=null, t0=0, tick=null, places={};
const W2P=(x,y)=>[(x-X0)*PPM,(Y1-y)*PPM];
const C={robot:"#38bdf8",goal:"#fb7185",path:"#34d399"};
const IDLE_HINT=`<div class="card-title">导航任务</div><div id="hint">在地图上<b>按住拖动</b>选择新的目标点，拖动方向=到达后的朝向。</div>`;

function fit(){
  const wrap=$("mapwrap");
  const scale=Math.min(1, wrap.clientWidth/img.naturalWidth, wrap.clientHeight/img.naturalHeight);
  img.style.width=img.naturalWidth*scale+"px"; img.style.height=img.naturalHeight*scale+"px";
  cv.width=img.naturalWidth; cv.height=img.naturalHeight;
  cv.style.width=img.style.width; cv.style.height=img.style.height;
}
img.onload=fit; window.onresize=fit; img.src="/map.png?"+Date.now();

/* ---------- drawing helpers (S keeps marker sizes proportional to map PPM) ---------- */
function rr(x,y,w,h,r){ctx.beginPath();ctx.moveTo(x+r,y);ctx.arcTo(x+w,y,x+w,y+h,r);
ctx.arcTo(x+w,y+h,x,y+h,r);ctx.arcTo(x,y+h,x,y,r);ctx.arcTo(x,y,x+w,y,r);ctx.closePath()}
function pill(x,y,text,color){ // rounded label centred above (x,y), clamped to canvas
  ctx.font=`600 ${12.5*S}px -apple-system,'PingFang SC',sans-serif`;
  const w=ctx.measureText(text).width+18*S;
  x=Math.max(w/2+2,Math.min(cv.width-w/2-2,x));
  ctx.fillStyle="rgba(7,12,18,.88)"; rr(x-w/2,y-26*S,w,22*S,7*S); ctx.fill();
  ctx.strokeStyle=color; ctx.globalAlpha=.55; ctx.lineWidth=1; ctx.stroke(); ctx.globalAlpha=1;
  ctx.fillStyle=color; ctx.textAlign="center"; ctx.textBaseline="middle"; ctx.fillText(text,x,y-15*S);
  ctx.textAlign="start"; ctx.textBaseline="alphabetic";
}
function arrowHead(x,y,ang,size,color){
  ctx.beginPath(); ctx.moveTo(x,y);
  ctx.lineTo(x-size*Math.cos(ang-.42),y-size*Math.sin(ang-.42));
  ctx.lineTo(x-size*Math.cos(ang+.42),y-size*Math.sin(ang+.42));
  ctx.closePath(); ctx.fillStyle=color; ctx.fill();
}
function headingRay(x,y,yawDeg,len,color){
  const a=-yawDeg*Math.PI/180, ex=x+len*Math.cos(a), ey=y+len*Math.sin(a);
  ctx.setLineDash([6*S,5*S]); ctx.strokeStyle=color; ctx.lineWidth=2*S; ctx.globalAlpha=.85;
  ctx.beginPath(); ctx.moveTo(x,y); ctx.lineTo(ex,ey); ctx.stroke();
  ctx.setLineDash([]); ctx.globalAlpha=1; arrowHead(ex,ey,a,9*S,color);
}

function drawPlan(){
  const pts=plan.path.map(p=>W2P(p[0],p[1]));
  ctx.lineJoin="round"; ctx.lineCap="round";
  ctx.strokeStyle="rgba(52,211,153,.16)"; ctx.lineWidth=10*S;
  ctx.beginPath(); pts.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1])); ctx.stroke();
  ctx.strokeStyle=C.path; ctx.lineWidth=3*S;
  ctx.beginPath(); pts.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1])); ctx.stroke();
  let acc=0,last=null; // direction chevrons along the route
  for(const q of pts){
    if(last){acc+=Math.hypot(q[0]-last[0],q[1]-last[1]);
      if(acc>52*S){const a=Math.atan2(q[1]-last[1],q[0]-last[0]);
        ctx.strokeStyle="rgba(6,78,59,.95)"; ctx.lineWidth=2.5*S;
        ctx.beginPath();
        ctx.moveTo(q[0]-8*S*Math.cos(a-.5),q[1]-8*S*Math.sin(a-.5)); ctx.lineTo(q[0],q[1]);
        ctx.lineTo(q[0]-8*S*Math.cos(a+.5),q[1]-8*S*Math.sin(a+.5)); ctx.stroke(); acc=0;}}
    last=q;
  }
  const s=W2P(plan.start[0],plan.start[1]);
  ctx.beginPath(); ctx.arc(s[0],s[1],7*S,0,7); ctx.strokeStyle=C.path; ctx.lineWidth=2.5*S; ctx.stroke();
  ctx.beginPath(); ctx.arc(s[0],s[1],2.6*S,0,7); ctx.fillStyle=C.path; ctx.fill();
  pill(s[0],s[1]-11*S,"起点","#6ee7b7");
}
function drawDrag(){
  if(!drag||drag.qx===undefined)return;
  ctx.setLineDash([5*S,5*S]); ctx.strokeStyle="rgba(251,113,133,.75)"; ctx.lineWidth=2*S;
  ctx.beginPath(); ctx.moveTo(drag.x,drag.y); ctx.lineTo(drag.qx,drag.qy); ctx.stroke(); ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(drag.x,drag.y,9*S,0,7); ctx.strokeStyle="rgba(251,113,133,.8)"; ctx.lineWidth=2*S; ctx.stroke();
}
function drawPlaces(){
  for(const [n,p] of Object.entries(places)){
    const q=W2P(p.x,p.y);
    ctx.beginPath(); ctx.arc(q[0],q[1],5*S,0,7);
    ctx.fillStyle="rgba(245,158,11,.95)"; ctx.fill();
    ctx.lineWidth=1.5*S; ctx.strokeStyle="rgba(254,240,138,.9)"; ctx.stroke();
    ctx.beginPath(); ctx.arc(q[0],q[1],1.8*S,0,7); ctx.fillStyle="#78350f"; ctx.fill();
    pill(q[0],q[1]+34*S,n,"#fbbf24");
  }
}
function drawGoal(t){
  const g=W2P(sel.x,sel.y);
  const r=S*(11+1.6*Math.sin(t/220)); // pulsing target rings
  ctx.strokeStyle=C.goal; ctx.lineWidth=2*S;
  ctx.globalAlpha=.9; ctx.beginPath(); ctx.arc(g[0],g[1],r,0,7); ctx.stroke();
  ctx.globalAlpha=.3; ctx.beginPath(); ctx.arc(g[0],g[1],r+7*S,0,7); ctx.stroke(); ctx.globalAlpha=1;
  headingRay(g[0],g[1],sel.yaw,42*S,C.goal);
  ctx.save(); ctx.shadowColor="rgba(251,113,133,.8)"; ctx.shadowBlur=12*S; // map pin
  ctx.beginPath(); ctx.moveTo(g[0],g[1]-3*S);
  ctx.quadraticCurveTo(g[0]+10*S,g[1]-16*S,g[0]+10*S,g[1]-23*S);
  ctx.arc(g[0],g[1]-24*S,10*S,0,Math.PI,false);
  ctx.quadraticCurveTo(g[0]-10*S,g[1]-16*S,g[0],g[1]-3*S);
  ctx.closePath();
  const pg=ctx.createLinearGradient(g[0]-10*S,g[1]-34*S,g[0]+10*S,g[1]);
  pg.addColorStop(0,"#fda4af"); pg.addColorStop(1,"#e11d48");
  ctx.fillStyle=pg; ctx.fill(); ctx.shadowBlur=0;
  ctx.strokeStyle="rgba(255,228,230,.85)"; ctx.lineWidth=1.2*S; ctx.stroke();
  ctx.beginPath(); ctx.arc(g[0],g[1]-24*S,3.6*S,0,7); ctx.fillStyle="#fff"; ctx.fill();
  ctx.restore();
  pill(g[0],g[1]-37*S,"目的地",C.goal);
}
function drawRobot(t){
  const p=W2P(pose[0],pose[1]), a=-pose[2]*Math.PI/180;
  ctx.beginPath(); ctx.arc(p[0],p[1],S*(17+2.2*Math.sin(t/280)),0,7);
  ctx.strokeStyle="rgba(56,189,248,.4)"; ctx.lineWidth=2*S; ctx.stroke();
  ctx.save(); ctx.translate(p[0],p[1]); ctx.rotate(a);
  ctx.shadowColor="rgba(56,189,248,.85)"; ctx.shadowBlur=13*S;
  const g=ctx.createLinearGradient(-10*S,0,15*S,0); g.addColorStop(0,"#0ea5e9"); g.addColorStop(1,"#7dd3fc");
  ctx.beginPath(); ctx.moveTo(15*S,0); ctx.lineTo(-10*S,10*S); ctx.lineTo(-5*S,0); ctx.lineTo(-10*S,-10*S); ctx.closePath();
  ctx.fillStyle=g; ctx.fill(); ctx.shadowBlur=0;
  ctx.strokeStyle="rgba(230,247,255,.9)"; ctx.lineWidth=1.4*S; ctx.stroke();
  ctx.restore();
  pill(p[0],p[1]-15*S,"机器人",C.robot);
}
function draw(t){
  ctx.clearRect(0,0,cv.width,cv.height);
  if(plan)drawPlan();
  if(drag)drawDrag();
  if(Object.keys(places).length)drawPlaces();
  if(sel)drawGoal(t);
  if(pose)drawRobot(t);
}
(function loop(){draw(performance.now());requestAnimationFrame(loop)})();

function ev(e){const r=cv.getBoundingClientRect();return{x:(e.clientX-r.left)*cv.width/r.width,y:(e.clientY-r.top)*cv.height/r.height};}
const busyPhase=()=>phase==="nav"||phase==="planning"||phase==="parsing";
cv.onmousedown=e=>{if(busyPhase())return;drag=ev(e);};
cv.onmousemove=e=>{if(!drag)return;const p=ev(e);drag.qx=p.x;drag.qy=p.y;};
cv.onmouseup=()=>{
  if(!drag)return;
  const g={x:X0+drag.x/PPM,y:Y1-drag.y/PPM,
    yaw:drag.qx!==undefined?Math.atan2(-(drag.qy-drag.y),drag.qx-drag.x)*180/Math.PI:(pose?pose[2]:0)};
  drag=null; selectGoal(g);
};
async function selectGoal(g){
  sel=g; phase="planning"; setPhase("规划中…");
  try{
    const r=await fetch("/preview",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({x:g.x,y:g.y,yaw_deg:g.yaw})});
    const d=await r.json();
    if(d.ok){plan=d;phase="ready";renderPanel(d);setPhase("待确认");}
    else{plan=null;phase="idle";sel=null;renderErr(d.error);setPhase("空闲");}
  }catch(err){phase="idle";sel=null;renderErr("预览请求失败: "+err);}
}
function setPhase(t){$("h-phase").textContent=t;}
function setChip(id,text,cls){
  const c=$(id); c.querySelector(".t").textContent=text;
  c.className="chip"+(cls?" "+cls:"");
}
function renderPanel(d){
  $("panel").innerHTML=`<div class="card-title">路线预览</div>
   <div class="row"><span>起点</span><b>(${d.start[0].toFixed(2)}, ${d.start[1].toFixed(2)}) · ${d.start[2].toFixed(0)}°</b></div>
   <div class="row"><span>目的地</span><b>(${d.goal[0].toFixed(2)}, ${d.goal[1].toFixed(2)}) · ${d.goal[2].toFixed(0)}°</b></div>
   <div class="row"><span>直线距离</span><b>${d.distance_m.toFixed(2)} m</b></div>
   <div class="row"><span>路径长度</span><b>${d.path_len_m.toFixed(2)} m · ${d.path.length} 点</b></div>
   <div class="row"><span>预计用时</span><b>约 ${d.est_s} s</b></div>
   <div class="actions">
   <button class="primary" onclick="startNav()">🚀 开始导航</button>
   <button class="ghost" onclick="resetSel()">取消</button></div>`;
}
function renderErr(msg){$("result").innerHTML=`<span class="err">❌ ${msg}</span>`;}
function resetSel(){sel=null;plan=null;phase="idle";draw(performance.now());$("panel").innerHTML=IDLE_HINT;setPhase("空闲");}
async function startNav(){
  if(phase!=="ready"||!sel)return;
  phase="nav";setPhase("导航中");
  $("panel").innerHTML=`<div class="card-title">导航执行</div>
   <div class="row"><span>目的地</span><b>(${plan.goal[0].toFixed(2)}, ${plan.goal[1].toFixed(2)})</b></div>
   <div class="row"><span>路径长度</span><b>${plan.path_len_m.toFixed(2)} m</b></div>
   <button class="danger" style="margin-top:14px" onclick="stopNav()">⛔ 停止导航</button>`;
  $("result").innerHTML=""; $("prog").style.display="flex";
  t0=Date.now();
  tick=setInterval(()=>{$("prog").textContent="已用时 "+Math.round((Date.now()-t0)/1000)+" s";},500);
  try{
    const r=await fetch("/goal",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({x:sel.x,y:sel.y,yaw_deg:sel.yaw})});
    const d=await r.json();
    clearInterval(tick);$("prog").style.display="none";
    const dt=((Date.now()-t0)/1000).toFixed(1);
    if(d.ok){
      $("result").innerHTML=`<span class="ok">✅ 导航成功到达</span>\\n用时 ${dt} s（本次实际 ${d.duration_s} s）\\n距离 ${d.distance_m} m`;
      setPhase("完成");
    }else{
      $("result").innerHTML=`<span class="err">❌ 导航失败</span>\\n原因：${d.error||"未知"}\\n用时 ${dt} s ｜ 距离 ${(d.distance_m===undefined?"-":d.distance_m)} m`;
      setPhase("失败");
    }
  }catch(err){clearInterval(tick);$("prog").style.display="none";renderErr("导航请求异常: "+err);setPhase("异常");}
  plan=null;
  setTimeout(()=>{if(phase==="完成"||phase==="失败"||phase==="异常"){phase="idle";sel=null;
    $("panel").innerHTML=IDLE_HINT;}},1500);
}
async function stopNav(){
  try{await fetch("/cancel",{method:"POST"});$("result").innerHTML='<span class="err">⛔ 已请求停止，会话收尾中…</span>';}
  catch(e){renderErr("停止请求失败: "+e);}
}
async function poll(){
  try{
    const d=await(await fetch("/state")).json();
    pose=d.pose;
    setChip("h-pose",pose?`(${pose[0].toFixed(2)}, ${pose[1].toFixed(2)}) ${pose[2].toFixed(0)}°`:"位置未知");
    setChip("h-health",d.healthy?(d.fresh?"定位健康":"定位过期"):"定位不健康",d.healthy?"ok":"warn");
    if(phase==="nav"&&!d.busy&&t0&&Date.now()-t0>6000){
      clearInterval(tick);$("prog").style.display="none";
      $("result").innerHTML='<span class="ok">导航会话已结束</span>';
      phase="idle";sel=null;plan=null;
      $("panel").innerHTML=IDLE_HINT;
    }
  }catch(e){setChip("h-health","服务离线","warn");}
}
/* ---------- natural language & places ---------- */
async function refreshPlaces(){
  try{
    const d=await(await fetch("/places")).json();
    places=d.places||{};
    $("placelist").innerHTML=Object.keys(places).map(n=>`<option value="${n}">`).join("");
  }catch(e){}
}
async function sendAsk(){
  const q=$("cmdin").value.trim();
  if(!q)return;
  if(busyPhase()){renderErr("导航进行中，请先停止或等其结束");return;}
  sel=null; plan=null;  // drop any pending unconfirmed selection
  phase="parsing"; setPhase("解析中…");
  try{
    const r=await fetch("/ask",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({text:q})});
    const d=await r.json();
    phase="idle";
    if(d.ok){
      $("cmdin").value="";
      $("result").innerHTML=`<span class="ok">🧠 ${d.source==="llm"?"大模型":"地点表"}解析：${d.name||`(${d.x.toFixed(2)}, ${d.y.toFixed(2)})`}${d.reply?` —— ${d.reply}`:""}</span>`;
      selectGoal({x:d.x,y:d.y,yaw:d.yaw_deg});
    }else if(d.candidates){
      $("result").innerHTML=`<span class="err">${d.error}</span><br>`+
        d.candidates.map(c=>`<button class="ghost" style="margin:4px;padding:6px 12px;font-size:13px" onclick="pickPlace('${c}')">${c}</button>`).join("");
      setPhase("空闲");
    }else{
      const names=Object.keys(places);
      renderErr(d.error+(names.length?`　可用地点：${names.join("、")}`:"（还没有地点，先点「标记当前位置」添加）"));
      setPhase("空闲");
    }
  }catch(err){phase="idle";setPhase("空闲");renderErr("解析请求失败: "+err);}
}
function pickPlace(n){
  if(busyPhase()){renderErr("导航进行中，请先停止或等其结束");return;}
  const p=places[n]; if(p) selectGoal({x:p.x,y:p.y,yaw:p.yaw});
}
async function markPlace(){
  const name=prompt("给当前位置起个名字（如：1705门口）：");
  if(!name)return;
  try{
    const r=await fetch("/place/save",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({name})});
    const d=await r.json();
    if(d.ok){
      $("result").innerHTML=`<span class="ok">📍 已标记「${d.name}」(${d.pose.x}, ${d.pose.y}, ${d.pose.yaw}°)</span>`;
      refreshPlaces();
    }else renderErr(d.error);
  }catch(err){renderErr("标记失败: "+err);}
}
$("cmdin").addEventListener("keydown",e=>{if(e.key==="Enter")sendAsk();});
refreshPlaces();
setInterval(poll,1500);poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype='application/json'):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith('/map.png'):
            self._send(200, render_map_png(), 'image/png')
        elif self.path == '/places':
            self._send(200, {'places': {k: {'x': v['x'], 'y': v['y'], 'yaw': v['yaw']}
                                        for k, v in PLACES.items()}})
        elif self.path == '/state':
            s = locator.snapshot()
            self._send(200, {'pose': [round(v, 3) for v in s['pose']] if s['pose'] else None,
                             'healthy': s['healthy'], 'fresh': s['fresh'], 'busy': busy['active']})
        else:
            page = (PAGE.replace('__PPM__', str(PPM)).replace('__X0__', f'{X0:.3f}')
                        .replace('__Y1__', f'{Y0 + H_M:.3f}').replace('__SCALE__', str(PPM / 20)))
            self._send(200, page.encode(), 'text/html')

    def _json_body(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            return json.loads(self.rfile.read(n))
        except Exception:
            return None

    def do_POST(self):
        if self.path == '/place/save':
            req = self._json_body()
            if req is None:
                return self._send(400, {'ok': False, 'error': '请求格式错误'})
            return self._send(200, place_save(req.get('name', '')))
        if self.path == '/place/resolve':
            req = self._json_body()
            if req is None:
                return self._send(400, {'ok': False, 'error': '请求格式错误'})
            return self._send(200, place_resolve(req.get('query', '')))
        if self.path == '/ask':
            req = self._json_body()
            if req is None:
                return self._send(400, {'ok': False, 'error': '请求格式错误'})
            return self._send(200, ask_goal(req.get('text', '')))
        if self.path == '/cancel':
            return self._send(200, cancel_nav())
        if self.path not in ('/goal', '/preview'):
            return self._send(404, {'error': 'not found'})
        if self.path == '/goal' and busy['active']:
            return self._send(409, {'ok': False, 'error': '已有导航在执行'})
        n = int(self.headers.get('Content-Length', 0))
        try:
            req = json.loads(self.rfile.read(n))
            x, y, yaw = float(req['x']), float(req['y']), float(req['yaw_deg'])
            if not all(map(math.isfinite, (x, y, yaw))):
                raise ValueError
        except Exception:
            return self._send(400, {'ok': False, 'error': '请求格式错误'})
        if self.path == '/preview':
            self._send(200, plan_preview(x, y, yaw))
        else:
            self._send(200, run_goal(x, y, yaw))


def main():
    global locator
    load_places()
    load_llm_cfg()
    rclpy.init()
    locator = Locator()
    threading.Thread(target=rclpy.spin, args=(locator,), daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', 8018), Handler)
    print('map clicker on http://0.0.0.0:8018 (Tailscale IP recommended)', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
