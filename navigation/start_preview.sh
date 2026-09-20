#!/usr/bin/env bash
set -e
nav="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_dir="/tmp/elf-navigation-${UID}"
mkdir -p "$runtime_dir"
chmod 700 "$runtime_dir"
exec 9>"$runtime_dir/preview.lock"
if ! flock -n 9; then
  echo "定位和 Nav2 已经在运行；请勿重复启动。若终端已丢失，先停止旧进程再重启。" >&2
  exit 1
fi
source "$nav/env.bash"
/usr/bin/python3 "$nav/tools/sync_robot_config.py"
exec ros2 launch "$nav/launch/navigation.launch.py" "$@"
