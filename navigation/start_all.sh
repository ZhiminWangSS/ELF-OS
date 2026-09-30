#!/usr/bin/env bash
# One-shot bring-up after a robot power cycle: lidar -> nav stack -> global
# localization -> Nav2 activation -> preflight -> web clicker.
# Usage:
#   bash start_all.sh              normal bring-up
#   bash start_all.sh --save-home  record the CURRENT pose as the parking pose
# With config/home_pose.yaml present, startup skips the ~1 min global search
# and inits directly at the parking pose (falls back to search automatically
# if the robot is not actually parked there).
set -o pipefail
NAV="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG=/tmp/elf-nav-preview.log
HOME_POSE="$NAV/config/home_pose.yaml"
source "$NAV/env.bash"
export ROS_DOMAIN_ID=42

say() { printf '\033[1;32m[%s]\033[0m %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\033[1;31m[FAIL]\033[0m %s\n' "$*" >&2; exit 1; }

if [ "${1:-}" = "--save-home" ]; then
  POSE=$(timeout 15 /usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, json, math, rclpy, yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init(); n = rclpy.create_node('save_home'); o = []; h = []
n.create_subscription(Odometry, '/odom', lambda m: o.append(m), 10)
n.create_subscription(Bool, '/navigation/localization_healthy', lambda m: h.append(m.data), 10)
end = time.monotonic() + 4
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
if not o or not all(h[-10:]):
    print(''); n.destroy_node(); rclpy.shutdown()
else:
    m = o[-1]; p = m.pose.pose.position; q = m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.z*q.z+q.y*q.y)))
    print(json.dumps({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw_deg': round(yaw, 1)}))
    n.destroy_node(); rclpy.shutdown()
PYEOF
)
  [ -n "$POSE" ] || die "定位不健康，无法记录停车位——先让系统正常运行再 --save-home"
  echo "# Parking pose recorded by start_all.sh --save-home" > "$HOME_POSE"
  echo "$POSE" | /usr/bin/python3 -c "import sys,yaml; d=yaml.safe_load(sys.stdin.read()); print(yaml.safe_dump(d), end='')" >> "$HOME_POSE"
  say "停车位已保存到 $HOME_POSE: $POSE"
  exit 0
fi

# --- 1. lidar driver -------------------------------------------------------
if ! pgrep -f livox_ros_driver2_node >/dev/null; then
  say "① 启动雷达驱动…"
  (cd "$NAV/../mapping/ws_livox" && source mapping_env.bash && \
   setsid nohup /usr/bin/python3 /opt/ros/foxy/bin/ros2 launch livox_ros_driver2 \
   msg_MID360_launch.py > /tmp/livox.log 2>&1 < /dev/null &)
  sleep 12
  pgrep -f livox_ros_driver2_node >/dev/null || die "雷达驱动未启动，看 /tmp/livox.log"
fi
say "① 雷达驱动: 运行中"

# IMU health gate: the known device fault throttles IMU to ~6.5 Hz; only a
# full power cycle fixes it, so detect it here instead of failing downstream.
IMU_HZ=$(/usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, rclpy
from sensor_msgs.msg import Imu
rclpy.init(); n = rclpy.create_node('start_all_imu'); c = []
n.create_subscription(Imu, '/livox/imu', lambda m: c.append(1), 200)
end = time.monotonic() + 3
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.005)
print(int(len(c) / 3)); n.destroy_node(); rclpy.shutdown()
PYEOF
)
if [ "${IMU_HZ:-0}" -lt 50 ]; then
  die "IMU 仅 ${IMU_HZ:-?} Hz（正常 200）——设备节流故障，请整机断电 10 秒重启后再跑本脚本"
fi
say "① IMU ${IMU_HZ} Hz: 正常"

# --- 2. nav stack ----------------------------------------------------------
# Reuse a running stack only if its LIO is actually tracking; a diverged LIO
# (pose tens of metres away / no healthy output) cannot recover without a
# restart, so kill it and bring up a clean one.
LIO_OK=$(timeout 12 /usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, rclpy, math
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init(); n = rclpy.create_node('start_all_lio'); o = []; h = []
n.create_subscription(Odometry, '/odom', lambda m: o.append(m), 10)
n.create_subscription(Bool, '/navigation/localization_healthy', lambda m: h.append(m.data), 10)
end = time.monotonic() + 4
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
ok = False
if o and h:
    p = o[-1].pose.pose.position
    ok = abs(p.x) < 50 and abs(p.y) < 50 and any(h)
print('yes' if ok else 'no'); n.destroy_node(); rclpy.shutdown()
PYEOF
)
if pgrep -f fastlio_mapping >/dev/null && [ "$LIO_OK" = "yes" ]; then
  say "② 导航栈: 已在运行且定位健康，跳过"
elif pgrep -f fastlio_mapping >/dev/null; then
  say "② 检测到定位已发散的旧栈，重启…"
  fp=$(ps -eo pid,cmd | grep fastlio_mapping | grep -v grep | grep -v "bash -c" | head -1 | awk '{print $1}')
  pgid=$(ps -o pgid= -p "$fp" | tr -d ' ')
  kill -INT -"$pgid" 2>/dev/null; sleep 7; kill -TERM -"$pgid" 2>/dev/null; sleep 3
  ps -eo pid,cmd | grep -E "fastlio_mapping|controller_server --ros|localization_adapter|prior_initializer|planner_server --ros|bt_navigator|map_server --ros|velocity_bridge|rviz_path" \
    | grep -v grep | grep -v "bash -c" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
  sleep 1; rm -f "/tmp/elf-navigation-${UID}/preview.lock"
fi
if ! pgrep -f fastlio_mapping >/dev/null; then
  say "② 启动导航栈…"
  rm -f "/tmp/elf-navigation-${UID}/preview.lock"
  (cd "$NAV" && setsid nohup bash start_preview.sh rviz:=false > "$LOG" 2>&1 < /dev/null &)
  sleep 22
  pgrep -f fastlio_mapping >/dev/null || die "导航栈未启动，看 $LOG"
fi
say "② 导航栈: 运行中"

# --- 3+4. localize -> verify LIO -> activate (retry whole stack on flaky windows)
LIO_TRACKING=$(timeout 12 /usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init(); n = rclpy.create_node('start_all_track'); h = []; o = []
n.create_subscription(Odometry, '/odom', lambda m: o.append(m), 10)
n.create_subscription(Bool, '/navigation/localization_healthy', lambda m: h.append(m.data), 10)
end = time.monotonic() + 6
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
ok = bool(o) and any(h) and abs(o[-1].pose.pose.position.x) < 50 and abs(o[-1].pose.pose.position.y) < 50
print('yes' if ok else 'no'); n.destroy_node(); rclpy.shutdown()
PYEOF
)

for ATTEMPT in 1 2 3; do
  [ "$LIO_TRACKING" = "yes" ] && break   # already healthy from a previous run
  if [ "$ATTEMPT" -gt 1 ]; then
    say "③ 第 $ATTEMPT 次尝试：重启导航栈…"
    fp=$(ps -eo pid,cmd | grep fastlio_mapping | grep -v grep | grep -v "bash -c" | head -1 | awk '{print $1}')
    if [ -n "$fp" ]; then
      pgid=$(ps -o pgid= -p "$fp" | tr -d ' ')
      kill -INT -"$pgid" 2>/dev/null; sleep 7; kill -TERM -"$pgid" 2>/dev/null; sleep 3
      ps -eo pid,cmd | grep -E "fastlio_mapping|controller_server --ros|localization_adapter|prior_initializer|planner_server --ros|bt_navigator|map_server --ros|velocity_bridge|rviz_path" \
        | grep -v grep | grep -v "bash -c" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
      sleep 1; rm -f "/tmp/elf-navigation-${UID}/preview.lock"
    fi
    (cd "$NAV" && setsid nohup bash start_preview.sh rviz:=false > "$LOG" 2>&1 < /dev/null &)
    sleep 22
    pgrep -f fastlio_mapping >/dev/null || die "导航栈未启动，看 $LOG"
  fi

  # Fast path: parking pose recorded via --save-home skips the ~1 min search.
  rm -f "$LOG.init_ok"
  if [ "$ATTEMPT" = "1" ] && [ -f "$HOME_POSE" ]; then
    read -r HX HY HYAW <<EOF
$(/usr/bin/python3 -c "import yaml; d=yaml.safe_load(open('$HOME_POSE')); print(d['x'], d['y'], d['yaw_deg'])" 2>/dev/null)
EOF
    if [ -n "${HX:-}" ]; then
      say "③ 停车位快速初始化 ($HX, $HY, ${HYAW}°)…"
      /usr/bin/python3 "$NAV/tools/set_initial_pose.py" --x "$HX" --y "$HY" --yaw-deg "$HYAW" >/dev/null 2>&1 || true
      for i in $(seq 1 15); do
        grep -aq "INITIALIZED" "$LOG" && break
        sleep 2
      done
      grep -aq "INITIALIZED" "$LOG" && touch "$LOG.init_ok"
      [ -f "$LOG.init_ok" ] || say "   停车位未通过（机器狗不在停车位？），回退全局搜索…"
    fi
  fi

  if [ ! -f "$LOG.init_ok" ]; then
  say "③ 全局搜索定位（第 $ATTEMPT 次，约 1-2 分钟）…"
  CANDIDATES=$(timeout 300 /usr/bin/python3 "$NAV/tools/global_localize.py" 2>/dev/null | /usr/bin/python3 -c "
import json,sys
lines=sys.stdin.read().splitlines()
try:
    start=next(i for i,l in enumerate(lines) if l.strip()=='[')
    for r in json.loads('\n'.join(lines[start:]))[:3]:
        print(r['base_x'],r['base_y'],r['base_yaw_deg'],r['overlap'])
except Exception: pass")
  if [ -z "$CANDIDATES" ]; then say "   全局搜索无结果，重试…"; continue; fi

  rm -f "$LOG.init_ok"
  echo "$CANDIDATES" | while read -r x y yaw ov; do
    say "   设初值 ($x, $y, ${yaw}°)…"
    /usr/bin/python3 "$NAV/tools/set_initial_pose.py" --x "$x" --y "$y" --yaw-deg "$yaw" >/dev/null 2>&1 || true
    for i in $(seq 1 12); do
      grep -aq "INITIALIZED" "$LOG" && break
      sleep 2
    done
    grep -aq "INITIALIZED" "$LOG" && touch "$LOG.init_ok" && break
  done
  [ -f "$LOG.init_ok" ] || { say "   ICP 未通过，重试…"; continue; }
  fi
  say "   ICP: INITIALIZED，验证 LIO 跟踪…"
  sleep 8
  LIO_TRACKING=$(timeout 12 /usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
rclpy.init(); n = rclpy.create_node('start_all_track'); h = []; o = []
n.create_subscription(Odometry, '/odom', lambda m: o.append(m), 10)
n.create_subscription(Bool, '/navigation/localization_healthy', lambda m: h.append(m.data), 10)
end = time.monotonic() + 6
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
ok = bool(o) and any(h) and abs(o[-1].pose.pose.position.x) < 50 and abs(o[-1].pose.pose.position.y) < 50
print('yes' if ok else 'no'); n.destroy_node(); rclpy.shutdown()
PYEOF
)
  [ "$LIO_TRACKING" = "yes" ] && break
  say "   LIO 未跟踪（设备节流窗口），重来…"
done
[ "$LIO_TRACKING" = "yes" ] || die "三次尝试后定位仍未稳定——建议整机断电 10 秒后重跑本脚本"
say "③ 定位: 初始化且跟踪正常"

# --- 4. Nav2 activation ----------------------------------------------------
for i in $(seq 1 20); do
  STATE=$(timeout 6 /opt/ros/foxy/bin/ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1)
  [ "$STATE" = "active [3]" ] && break
  sleep 3
done
[ "$STATE" = "active [3]" ] || die "Nav2 未激活（$STATE）——等几秒重跑本脚本"
say "④ Nav2: 全部激活"

# --- 5. preflight ----------------------------------------------------------
READY=$(timeout 60 /usr/bin/python3 "$NAV/tools/preflight.py" 2>/dev/null | /usr/bin/python3 -c "import json,sys; print(json.load(sys.stdin)['ready_for_controlled_motion_test'])" 2>/dev/null)
if [ "$READY" = "True" ]; then
  say "⑤ preflight: 全绿"
else
  say "⑤ preflight: 未通过（多为遥控器未开）——不影响地图显示/规划；真机行走前请开遥控器并重跑"
fi

# --- 6. web clicker --------------------------------------------------------
if ! curl -s -m 3 http://127.0.0.1:8018/state >/dev/null 2>&1; then
  say "⑥ 启动网页选点器…"
  (cd "$NAV" && setsid nohup /usr/bin/python3 tools/map_clicker.py > /tmp/map_clicker.log 2>&1 < /dev/null &)
  sleep 4
fi
curl -s -m 3 http://127.0.0.1:8018/state >/dev/null 2>&1 || die "选点器未响应，看 /tmp/map_clicker.log"
POSE=$(curl -s -m 3 http://127.0.0.1:8018/state)
say "⑥ 选点器: 在线 $POSE"

# --- 7. self-healing watchdog (dies with the robot reboot, revive it) ------
if ! pgrep -f "autoheal\.sh" >/dev/null; then
  setsid nohup bash "$NAV/autoheal.sh" >/dev/null 2>&1 < /dev/null &
  sleep 1
  pgrep -f "autoheal\.sh" >/dev/null && say "⑦ 自愈守护: 已启动" || say "⑦ 自愈守护: 启动失败（可手动: setsid nohup bash $NAV/autoheal.sh &）"
else
  say "⑦ 自愈守护: 已在运行"
fi
say "✅ 全部就绪 —— 浏览器打开 http://<机器狗IP>:8018 （Tailscale: 100.78.79.38:8018）"
