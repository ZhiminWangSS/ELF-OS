#!/usr/bin/env bash
# Watchdog: keep the navigation stack alive despite the intermittent lidar
# IMU-throttle fault. Every 30 s it checks LIO sanity (odometry present and
# pose not diverged); after two consecutive bad checks it re-runs start_all.sh
# (which is idempotent: healthy steps are skipped, dead ones redone).
# Usage: setsid nohup bash autoheal.sh > /tmp/autoheal.log 2>&1 &
NAV="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$NAV/env.bash" 2>/dev/null
export ROS_DOMAIN_ID=42

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" >> /tmp/autoheal.log; }

check_lio() {
  timeout 12 /usr/bin/python3 - <<'PYEOF' 2>/dev/null
import time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('autoheal_check'); o = []
n.create_subscription(Odometry, '/odom', lambda m: o.append(m), 10)
end = time.monotonic() + 5
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
ok = False
if o:
    p = o[-1].pose.pose.position
    ok = abs(p.x) < 50 and abs(p.y) < 50
print('yes' if ok else 'no'); n.destroy_node(); rclpy.shutdown()
PYEOF
}

BAD=0
log "watchdog started (pid $$)"
while true; do
  STATE=$(check_lio)
  if [ "$STATE" = "yes" ]; then
    BAD=0
  else
    BAD=$((BAD + 1))
    log "LIO 异常 ($STATE) 第 $BAD/2 次"
    if [ "$BAD" -ge 2 ]; then
      log "触发自动恢复: start_all.sh"
      BAD=0
      bash "$NAV/start_all.sh" >> /tmp/autoheal.log 2>&1
      log "start_all 退出码 $? （成功=定位已恢复）"
    fi
  fi
  sleep 30
done
