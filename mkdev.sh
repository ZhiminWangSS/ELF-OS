#!/usr/bin/env bash
# 为一名开发者建立独立开发 worktree：/home/unitree/dev/<名字>/ELF-OS
# 用法: bash mkdev.sh <名字> <git邮箱>
# 例:   bash mkdev.sh zhimin someone@example.com
set -euo pipefail
SELF="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME=/home/unitree/pingandog/ELF-OS

NAME="${1:?用法: bash mkdev.sh <名字> <git邮箱>}"
EMAIL="${2:?用法: bash mkdev.sh <名字> <git邮箱>}"
BR="dev/$NAME"
DEST="/home/unitree/dev/$NAME/ELF-OS"

[ -e "$DEST" ] && { echo "已存在: $DEST（重装请先: git -C $RUNTIME worktree remove $DEST）"; exit 1; }

cd "$SELF"
git fetch origin
BASE=$(git rev-parse --verify -q origin/main || git rev-parse HEAD)

if git show-ref --verify --quiet "refs/heads/$BR"; then
  git worktree add "$DEST" "$BR"
else
  git worktree add -b "$BR" "$DEST" "$BASE"
fi

# 各 worktree 独立署名，提交不会都算到同一个人头上
git -C "$DEST" config --worktree user.name "$NAME"
git -C "$DEST" config --worktree user.email "$EMAIL"

# 共享运行资产：被 gitignore 的预编译产物/地图，软链到运行目录的那份。
# 软链接本身匹配不到 .gitignore 的目录规则(xxx/)，统一加进本地 exclude，
# 防止误提交指向机器人绝对路径的链接。
link() {
  [ -e "$1" ] || return 0
  [ -e "$2" ] || ln -s "$1" "$2"
}
EXCLUDE="$(cd "$SELF" && git rev-parse --git-common-dir)/info/exclude"
for p in /Livox-SDK2 /mapping/reloc2_ws /mapping/ws_livox/build /mapping/ws_livox/install /navigation/maps; do
  grep -qx "$p" "$EXCLUDE" 2>/dev/null || echo "$p" >> "$EXCLUDE"
done
mkdir -p "$DEST/mapping/ws_livox" "$DEST/navigation"
link "$RUNTIME/mapping/reloc2_ws"        "$DEST/mapping/reloc2_ws"
link "$RUNTIME/mapping/ws_livox/build"   "$DEST/mapping/ws_livox/build"
link "$RUNTIME/mapping/ws_livox/install" "$DEST/mapping/ws_livox/install"
link "$RUNTIME/navigation/maps"          "$DEST/navigation/maps"
link "$RUNTIME/Livox-SDK2"               "$DEST/Livox-SDK2"

echo "✅ 开发目录: $DEST"
echo "   分支: $BR (基于 $(git -C "$DEST" log --oneline -1))"
echo "   推送: git push origin $BR   合并后在机器人运行目录执行: bash deploy.sh"
