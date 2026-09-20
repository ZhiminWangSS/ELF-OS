#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/foxy/setup.bash
if [[ ! -f /usr/local/include/livox_lidar_api.h || ! -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]]; then
  echo "缺少 Livox-SDK2，请按 README_STEP1.md 安装官方 SDK。" >&2
  exit 1
fi
if [[ ! -f "$workspace_dir/src/livox_ros_driver2/package.xml" ]]; then
  echo "缺少 src/livox_ros_driver2 源码链接，请按 README_STEP1.md 恢复。" >&2
  exit 1
fi
cd "$workspace_dir"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon build --packages-select livox_ros_driver2 --executor sequential \
  --cmake-args -DROS_EDITION=ROS2 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
