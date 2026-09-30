#!/usr/bin/env bash
# Open the viewer without starting a second localization/navigation stack.
set -e
nav_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo '当前终端没有图形桌面连接。请在远程桌面的终端中运行此脚本。' >&2
  exit 1
fi
source "$nav_dir/env.bash"
exec rviz2 -d "$nav_dir/config/navigation.rviz" "$@"
