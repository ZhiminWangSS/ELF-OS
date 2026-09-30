#!/usr/bin/env bash
# 把 GitHub main 部署到运行目录 /home/unitree/pingandog/ELF-OS。
# 只在运行目录使用；开发请到 /home/unitree/dev/<名字>/ELF-OS。
set -euo pipefail
RUNTIME=/home/unitree/pingandog/ELF-OS
cd "$RUNTIME"
git fetch origin

if [ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)" ]; then
  echo "已是最新: $(git log --oneline -1)"
  exit 0
fi
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "运行目录有未提交改动，拒绝部署。请在开发目录处理、合并后重试。" >&2
  exit 1
fi

echo "将部署以下提交:"
git log --oneline HEAD..origin/main
git merge --ff-only origin/main
echo "✅ 部署完成"
echo "   重启服务: bash navigation/start_all.sh"
echo "   涉及 C++ 改动时先构建: bash navigation/build.sh"
