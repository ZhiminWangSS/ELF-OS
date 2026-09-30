#!/usr/bin/env bash
# Keep the localization/Nav2 process tree independent of the calling terminal.
# This starts no SDK worker and sends no navigation goal.
set -euo pipefail
nav_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if systemctl --user is-active --quiet elf-navigation.service; then
  echo 'Navigation service already active.'
  exit 0
fi
systemctl --user reset-failed elf-navigation.service 2>/dev/null || true
exec systemd-run --user --unit=elf-navigation \
  --property="WorkingDirectory=$nav_dir/.." \
  /bin/bash "$nav_dir/start_preview.sh" rviz:=false
