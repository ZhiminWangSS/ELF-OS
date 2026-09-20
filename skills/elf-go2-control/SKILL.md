---
name: elf-go2-control
description: 在 Unitree Go2 的 NX 上通过 ELF-OS 获取实时 RGB、读取状态、执行受限短步并停止和核验。用于实际观察或移动机器狗；完整像素导航和 Nav2 接入仍按架构文档分阶段实现。
---

# ELF Go2 基础能力

代码入口为本仓库 `ELF-OS/control/go2.py`。当前部署绝对路径：
`/home/unitree/pingandog/ELF-OS/control/go2.py`。
搬迁时从此 SKILL.md 的真实路径向上两级找到 ELF-OS；不要调用旧 `scripts/go2_nav_step.py` 绕过新执行器检查。

## 使用范围

模型在对话中读取图片并决定短程动作，CLI 执行 SDK 调用。改代码、设计架构或读取本 skill 本身不构成移动授权；当前任务已明确要求移动时，不重复索要笼统许可。复用授权前确认本次方向/范围属于任务，实时环境仍允许。

只做 `observe/state/step/stop`。不切换 Wi-Fi、不修改网络、不自动站起、不关闭 sport_mode、不调用 LowCmd/关节指令。skill 并不实现深度推断、可通行性检测或自动导航。

## 命令

```bash
python3 /home/unitree/pingandog/ELF-OS/control/go2.py build
python3 /home/unitree/pingandog/ELF-OS/control/go2.py observe
python3 /home/unitree/pingandog/ELF-OS/control/go2.py state
python3 /home/unitree/pingandog/ELF-OS/control/go2.py step --vx 0.2 --seconds 1.25
python3 /home/unitree/pingandog/ELF-OS/control/go2.py stop
```

`step` 上例是 dry-run，无运动。`stop` 是真实停止，独立于 observation 和 --execute。

## 实际短步流程

1. 先 observe，用图像工具打开返回的 JPEG；检查行进范围中的人员、线缆、障碍和落差，不能只看 RPC 成功。状态需新鲜、mode 1、error_code 0 或 2（2 必须记录警告）；确认站稳且无外接线缆牵制。证据不足时指出具体疑点，补充观察或请现场处理。
2. 选一个已授权的短步。原图无明确可走区域时不猜测距离；固定为 2 m 的 range_obstacle 不作为空闲证明。短步没有自动避障，周围存在动态风险时不发送。
3. 使用返回的真实 observation 文件路径（60 秒内，同 NX 启动和同接口），执行前说明速度和时长。例如：

   ```bash
   python3 /home/unitree/pingandog/ELF-OS/control/go2.py step --vx 0.2 --seconds 1.25 --observation /absolute/path/from/observe.observation.json --execute
   ```

   单次上限为前进 0.20 m/s、1.25 s；旋转需 `--vx 0 --yaw-rate ±0.35`，只在用户授权和旋转足迹已检查时使用。不能合并转向和前进。转向接口已实现，尚未实机验收。
4. 等待同一个进程完成；不要因等待超时另起重复动作。中断/失败需确认停止日志；必要时独立 stop。不得自行扩大已授权的错误码例外、拉长时长或调用旧 helper 重试失败运动。
5. 动作后 observe，比较位置、姿态和图像；报告测得/估计的位移与名义距离差别，保留日志。stop code=0 是 RPC 确认，并非严格静止证明。遇到 0、2 以外的 error_code 或停止失败时报告并终止本次动作链。

本次任务只做一个动作时，到此结束；连续任务也必须重新观察判断，不能从一次成功自动衍生无限循环。

## 已知部署事实

- NX eth0 `192.168.123.18/24`，SDK domain 0；wlan0 可保持外网连接。
- CLI 为子进程单独选择 Unitree SDK DDS 库，避免 ROS Foxy libddsc ABI 冲突；不要全局修改 shell 库路径。
- 旧执行器完成过一次名义 25 cm、状态估计约 31 cm 的前进，证据在 `control/captures/step_verification/`。
- error_code=2 的官方含义未核实。2026-09-16 用户明确授权忽略此警告；当前执行器仅对精确值 2 记录 guard_warning 并继续，其他非零码仍停止。该部署例外不代表宇树确认它无害；状态新鲜度、mode、姿态、限速和停止检查仍有效。
- 当前状态检查、停止与本地锁不防止其他 SDK 客户端抢控制，尚无硬件独立 watchdog 或遥控接管适配。没有深度、标定和 costmap 时，不声称完成自主避障。

## 报错排查

先保留原始报错与本次 `runs/` 路径，区分三种码：`state.error_code` 是机器人广播状态，`move/stop.code` 是 SDK RPC 返回值，`result.json.returncode` 是执行器进程退出码。不能把它们的数字互相解释。CLI 参数解析失败也会退出 2，此时通常有 `usage:`，且尚未调用 SDK。

新日志中的 `guard_rejected` 会指出 `before_move/during_move/after_stop` 阶段、具体原因和机器人原始状态码；`result.json` 分开汇总 `guard_rejections` 与 `rpc_failures`。旧日志需结合 `stdout.jsonl` 和 `stderr.log` 查看；机器人状态码 2 不等于 skill 加载失败。排查细节与已核实证据见 [错误码说明](../../control/README.md#错误码排查)。

## 深入阅读

- 构建、命令、日志及限制：[基础模块说明](../../control/README.md)。
- 规划用户框图或实现下一阶段时，读 [导航架构](../../docs/navigation-architecture.md) 与 [接口契约](../../docs/navigation-contracts.md)。
- 只在对应后端完成后新增 `elf-pixel-goal`、`elf-nav2-navigation`、`elf-semantic-exploration`；目前这些是设计职责，不是可调用工具。
