#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/foxy/setup.bash
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "驱动尚未编译，请先在 $workspace_dir 中运行 colcon build。" >&2
  exit 1
fi
source "$workspace_dir/mapping_env.bash"
python3 "$workspace_dir/check_mid360.py"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/ros}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py "$@"
