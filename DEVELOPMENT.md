# 多人隔离开发流程

机器人（robotdog）上多人同时开发，不再共用一个工作区。代码隔离靠
**git worktree**：仓库只有一份 `.git`，每人一个独立工作目录、独立分支，
通过 GitHub 汇合，机器人运行目录只做部署。

## 目录结构

| 目录 | 角色 |
| --- | --- |
| `/home/unitree/pingandog/ELF-OS` | **运行目录**：机器人服务从这里跑。只部署，不开发，不切分支 |
| `/home/unitree/dev/<名字>/ELF-OS` | **开发目录**：每人在自己的目录里改代码，分支 `dev/<名字>` |
| GitHub `ZhiminWangSS/ELF-OS` | 集成通道：`dev/*` 分支合入 `main` |

## 建立自己的开发目录（每人一次）

```bash
cd /home/unitree/pingandog/ELF-OS
bash mkdev.sh <你的名字> <你的git邮箱>
# 例: bash mkdev.sh zhimin someone@example.com
```

之后日常开发都在 `/home/unitree/dev/<名字>/ELF-OS` 里进行。

## 日常流程

```text
① 开发目录里改代码、提交           cd /home/unitree/dev/<名字>/ELF-OS
② 推送自己的分支                   git push origin dev/<名字>
③ 在 GitHub 上把 dev/<名字> 合入 main（发 PR 或网页合并）
④ 回机器人部署                     bash /home/unitree/pingandog/ELF-OS/deploy.sh
⑤ 重启服务                         bash navigation/start_all.sh
⑥ 同步别人的合并（开发目录里）      git fetch origin && git merge origin/main
```

## 真机测试须知

- 测试自己目录里的导航栈前，先停掉线上栈（`pkill -f fastlio_mapping`
  等导航进程），两套栈会抢同一雷达、话题和端口（网页选点器 8018）。
- 开发目录里的 `mapping/reloc2_ws`、`mapping/ws_livox/build|install`、
  `navigation/maps`、`Livox-SDK2` 是指向运行目录的**软链接**（共享的
  预编译产物和地图），不要删除；C++ 改动需要时用
  `bash navigation/build.sh` 重新构建。
- 导航环境变量：`source navigation/env.bash`，`ROS_DOMAIN_ID=42`。
- 遥控器介入会切到 locomotion 模式，程序指令会被护栏拒绝，属正常行为。

## 红线（防互相踩踏）

1. **不要在运行目录改代码或切分支**——机器人服务正在从这里跑，
   文件一变线上行为就变。改代码一律去自己的开发目录。
2. **不要直接推 main**——改动先进 `dev/<名字>` 分支，在 GitHub 上合并。
3. **每人只用自己名字的分支和目录**，不去别人的开发目录里改东西。
