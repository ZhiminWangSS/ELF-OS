#!/usr/bin/env python3
"""Web map click-to-navigate for supervised short tests.

Serves the prior map on http://<robot>:8018. A click-drag picks (x, y, yaw);
the server validates localization health, goal free-space clearance and the
short-test distance cap, then runs tools/hardware_test.py --execute with the
picked goal (same supervised flow as the shell workflow). No auth: only bind
to interfaces you trust (Tailscale)."""
import json, math, os, subprocess, threading, time
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
PPM = 20  # rendered pixels per metre

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


def render_map_png():
    lut = np.zeros(256, dtype=np.uint8)
    lut[254] = 235   # free -> light
    lut[205] = 120   # unknown -> gray
    lut[0] = 10      # occupied -> near black
    img = Image.fromarray(lut[grid[::-1]]).convert('RGB')
    img = img.resize((int(W_M * PPM), int(H_M * PPM)), Image.NEAREST)
    d = ImageDraw.Draw(img, 'RGBA')
    for meter in range(0, int(max(W_M, H_M)) + 1, 2):
        px = int((meter - X0) * PPM) if X0 <= meter <= X0 + W_M else -1
        py = int((Y0 + H_M - meter) * PPM) if Y0 <= meter <= Y0 + H_M else -1
        major = meter % 10 == 0
        if px >= 0:
            d.line([(px, 0), (px, img.height)], fill=(0, 116, 217, 200 if major else 70), width=2 if major else 1)
        if py >= 0:
            d.line([(0, py), (img.width, py)], fill=(0, 116, 217, 200 if major else 70), width=2 if major else 1)
        if major:
            if px >= 0:
                d.text((px + 4, 6), str(meter), fill=(0, 90, 180, 255))
            if py >= 0:
                d.text((6, py + 4), str(meter), fill=(0, 90, 180, 255))
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


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>ELF 地图选点导航</title>
<style>
body{font-family:"PingFang SC","Microsoft YaHei",sans-serif;background:#0e1216;color:#d8dee6;margin:0;display:flex;flex-direction:column;height:100vh}
header{display:flex;align-items:center;gap:10px;padding:8px 14px;background:#141b22;border-bottom:1px solid #26303a}
header h1{font-size:16px;margin:0;color:#8fb4d9}
.chip{padding:3px 10px;border-radius:6px;background:#202a34;font-size:13px}
.chip.ok{background:#23402a;color:#9fe0ae}.chip.warn{background:#4a2a2a;color:#ff9c9c}
#main{display:flex;flex:1;min-height:0}
#mapwrap{flex:1;position:relative;overflow:hidden;background:#000}
#map{position:absolute;left:0;top:0;transform-origin:top left}
#cv{position:absolute;left:0;top:0;cursor:crosshair}
aside{width:320px;background:#141b22;border-left:1px solid #26303a;padding:14px;display:flex;flex-direction:column;gap:10px;overflow-y:auto}
aside h2{font-size:14px;margin:0;color:#8fb4d9}
table{width:100%%;border-collapse:collapse;font-size:13px}
td{padding:4px 6px;border-bottom:1px solid #1e2833}
td:first-child{color:#7d8b99}
button{padding:12px;border:none;border-radius:8px;font-size:16px;cursor:pointer;background:#2b5f2f;color:#e6ffe6}
button:hover{background:#357438}
button.gray{background:#33414e;color:#c6d2dd}
button.gray:hover{background:#3d4e5d}
button.red{background:#7a2222;color:#ffd6d6}
button.red:hover{background:#963030}
button:disabled{opacity:.4;cursor:default}
#result{font-size:13px;line-height:1.6;white-space:pre-wrap}
.ok{color:#9fe0ae}.err{color:#ff9c9c}
#hint{font-size:12px;color:#7d8b99;line-height:1.7}
</style></head><body>
<header><h1>ELF 地图选点导航</h1>
 <span class="chip" id="h-pose">位置获取中…</span>
 <span class="chip" id="h-health">…</span>
 <span class="chip" id="h-phase">空闲</span>
</header>
<div id="main">
 <div id="mapwrap"><img id="map"><canvas id="cv"></canvas></div>
 <aside>
  <h2>导航任务</h2>
  <div id="panel"><div id="hint">在地图上<b>按住拖动</b>选择目标点与到达朝向：
起点=目标位置，拖动方向=到达后的朝向。<br><br>
松开后系统自动规划路线并显示预览，<b>按下「开始导航」才会真正出发</b>。<br><br>
单段上限 15 m ｜ 遥控器随时接管 ｜ 急停可用</div></div>
  <div id="result"></div>
  <div id="prog" style="display:none;font-size:14px;color:#ffd27c"></div>
 </aside>
</div>
<script>
const PPM=%(PPM)d, X0=%(X0).3f, Y1=%(Y1).3f;
const $=id=>document.getElementById(id);
const img=$("map"), cv=$("cv"), ctx=cv.getContext("2d");
let pose=null, phase="idle", sel=null, drag=null, plan=null, t0=0, tick=null;
const W2P=(x,y)=>[(x-X0)*PPM,(Y1-y)*PPM];

function fit(){
  const wrap=$("mapwrap");
  const scale=Math.min(1, wrap.clientWidth/img.naturalWidth, wrap.clientHeight/img.naturalHeight);
  img.style.width=img.naturalWidth*scale+"px"; img.style.height=img.naturalHeight*scale+"px";
  cv.width=img.naturalWidth; cv.height=img.naturalHeight;
  cv.style.width=img.style.width; cv.style.height=img.style.height;
  draw();
}
img.onload=fit; window.onresize=fit; img.src="/map.png?"+Date.now();

function draw(){
  ctx.clearRect(0,0,cv.width,cv.height);
  if(plan){ // planned route
    ctx.strokeStyle="#3fd15f"; ctx.lineWidth=3; ctx.setLineDash([]);
    ctx.beginPath();
    plan.path.forEach((p,i)=>{const a=W2P(p[0],p[1]); i?ctx.lineTo(a[0],a[1]):ctx.moveTo(a[0],a[1]);});
    ctx.stroke();
    const s=W2P(plan.start[0],plan.start[1]);
    ctx.strokeStyle="#3fd15f"; ctx.lineWidth=2; ctx.beginPath(); ctx.arc(s[0],s[1],8,0,7); ctx.stroke();
    ctx.fillStyle="#9fe0ae"; ctx.font="12px sans-serif"; ctx.fillText("出发点",s[0]+10,s[1]-8);
  }
  if(sel){
    const g=W2P(sel.x,sel.y);
    ctx.strokeStyle="#f44336"; ctx.lineWidth=3; ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(g[0],g[1],10,0,7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(g[0],g[1]);
    ctx.lineTo(g[0]+30*Math.cos(-sel.yaw*Math.PI/180),g[1]+30*Math.sin(-sel.yaw*Math.PI/180)); ctx.stroke();
    ctx.fillStyle="#ff9c9c"; ctx.font="12px sans-serif"; ctx.fillText("目的地",g[0]+12,g[1]-10);
  }
  if(pose){
    const p=W2P(pose[0],pose[1]);
    ctx.fillStyle="#2196f3"; ctx.beginPath(); ctx.arc(p[0],p[1],8,0,7); ctx.fill();
    ctx.strokeStyle="#2196f3"; ctx.lineWidth=3; ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(p[0],p[1]);
    ctx.lineTo(p[0]+24*Math.cos(-pose[2]*Math.PI/180),p[1]+24*Math.sin(-pose[2]*Math.PI/180)); ctx.stroke();
  }
}
function ev(e){const r=cv.getBoundingClientRect();return{x:(e.clientX-r.left)*cv.width/r.width,y:(e.clientY-r.top)*cv.height/r.height};}
cv.onmousedown=e=>{if(phase!=="idle")return;drag=ev(e);};
cv.onmousemove=e=>{if(!drag)return;const p=ev(e);drag.qx=p.x;drag.qy=p.y;};
cv.onmouseup=async()=>{
  if(!drag)return;
  const g={x:X0+drag.x/PPM,y:Y1-drag.y/PPM,
    yaw:drag.qx!==undefined?Math.atan2(-(drag.qy-drag.y),drag.qx-drag.x)*180/Math.PI:(pose?pose[2]:0)};
  drag=null; sel=g; phase="planning"; setPhase("规划中…"); draw();
  try{
    const r=await fetch("/preview",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({x:g.x,y:g.y,yaw_deg:g.yaw})});
    const d=await r.json();
    if(d.ok){plan=d;phase="ready";renderPanel(d);setPhase("待确认");}
    else{plan=null;phase="idle";sel=null;draw();renderErr(d.error);setPhase("空闲");}
  }catch(err){phase="idle";sel=null;draw();renderErr("预览请求失败: "+err);}
};
function setPhase(t){$("h-phase").textContent=t;}
function renderPanel(d){
  $("panel").innerHTML=`<table>
   <tr><td>出发点</td><td>(${d.start[0].toFixed(2)}, ${d.start[1].toFixed(2)}) 朝向 ${d.start[2].toFixed(0)}°</td></tr>
   <tr><td>目的地</td><td>(${d.goal[0].toFixed(2)}, ${d.goal[1].toFixed(2)}) 朝向 ${d.goal[2].toFixed(0)}°</td></tr>
   <tr><td>直线距离</td><td>${d.distance_m.toFixed(2)} m</td></tr>
   <tr><td>规划路径</td><td>${d.path_len_m.toFixed(2)} m（${d.path.length} 个路径点）</td></tr>
   <tr><td>预计用时</td><td>约 ${d.est_s} s</td></tr></table>
   <div style="display:flex;gap:8px;margin-top:12px">
   <button style="flex:2" onclick="startNav()">🚀 开始导航</button>
   <button class="gray" style="flex:1" onclick="resetSel()">取消</button></div>`;
}
function renderErr(msg){$("result").innerHTML=`<span class="err">❌ ${msg}</span>`;}
function resetSel(){sel=null;plan=null;phase="idle";draw();$("panel").innerHTML=$("panel").dataset.idle||$("panel").innerHTML;setPhase("空闲");}
async function startNav(){
  if(phase!=="ready"||!sel)return;
  phase="nav";setPhase("导航中");
  $("panel").innerHTML=`<table><tr><td>目的地</td><td>(${plan.goal[0].toFixed(2)}, ${plan.goal[1].toFixed(2)})</td></tr>
   <tr><td>规划路径</td><td>${plan.path_len_m.toFixed(2)} m</td></tr></table>
   <button class="red" style="width:100%%;margin-top:12px" onclick="stopNav()">⛔ 停止导航</button>`;
  $("result").innerHTML=""; $("prog").style.display="block";
  t0=Date.now();
  tick=setInterval(()=>{$("prog").textContent="⏱ 已用时 "+Math.round((Date.now()-t0)/1000)+" s";},500);
  try{
    const r=await fetch("/goal",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({x:sel.x,y:sel.y,yaw_deg:sel.yaw})});
    const d=await r.json();
    clearInterval(tick);$("prog").style.display="none";
    const dt=((Date.now()-t0)/1000).toFixed(1);
    if(d.ok){
      $("result").innerHTML=`<span class="ok">✅ 导航成功到达</span>\n用时 ${dt} s（本次实际 ${d.duration_s} s）\n距离 ${d.distance_m} m`;
      setPhase("完成");
    }else{
      $("result").innerHTML=`<span class="err">❌ 导航失败</span>\n原因：${d.error||"未知"}\n用时 ${dt} s ｜ 距离 ${(d.distance_m===undefined?"-":d.distance_m)} m`;
      setPhase("失败");
    }
  }catch(err){clearInterval(tick);$("prog").style.display="none";renderErr("导航请求异常: "+err);setPhase("异常");}
  plan=null;draw();
  setTimeout(()=>{if(phase==="完成"||phase==="失败"||phase==="异常"){phase="idle";sel=null;draw();
    $("panel").innerHTML='<div id="hint">在地图上<b>按住拖动</b>选择新的目标点</div>';}},1500);
}
async function stopNav(){
  try{await fetch("/cancel",{method:"POST"});$("result").innerHTML='<span class="err">⛔ 已请求停止，会话收尾中…</span>';}
  catch(e){renderErr("停止请求失败: "+e);}
}
async function poll(){
  try{
    const d=await(await fetch("/state")).json();
    pose=d.pose;
    $("h-pose").textContent=pose?`(${pose[0].toFixed(2)}, ${pose[1].toFixed(2)}) ${pose[2].toFixed(0)}°`:"位置未知";
    $("h-health").textContent=d.healthy?(d.fresh?"定位健康":"定位过期"):"定位不健康";
    $("h-health").className="chip "+(d.healthy?"ok":"warn");
    if(phase==="nav"&&!d.busy&&t0&&Date.now()-t0>6000){
      clearInterval(tick);$("prog").style.display="none";
      $("result").innerHTML='<span class="ok">导航会话已结束</span>';
      phase="idle";sel=null;plan=null;draw();
      $("panel").innerHTML='<div id="hint">在地图上<b>按住拖动</b>选择新的目标点</div>';
    }
    draw();
  }catch(e){$("h-health").textContent="服务离线";$("h-health").className="chip warn";}
}
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
        elif self.path == '/state':
            s = locator.snapshot()
            self._send(200, {'pose': [round(v, 3) for v in s['pose']] if s['pose'] else None,
                             'healthy': s['healthy'], 'fresh': s['fresh'], 'busy': busy['active']})
        else:
            self._send(200, (PAGE % {'PPM': PPM, 'X0': X0, 'Y1': Y0 + H_M, 'H_M': H_M}).encode(), 'text/html')

    def do_POST(self):
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
    rclpy.init()
    locator = Locator()
    threading.Thread(target=rclpy.spin, args=(locator,), daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', 8018), Handler)
    print('map clicker on http://0.0.0.0:8018 (Tailscale IP recommended)', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
