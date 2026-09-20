# ELF-OS Go2 基础能力

统一入口：`go2.py`。当前提供实时 RGB＋状态观察、只读状态、单次受限速度、停止。源码和构建都在此目录，使用外部官方 Unitree C++ SDK；无需外层 navila_bridge、旧 build/go2_comm_tests 或 ROS 运行环境。

## 构建和只读使用

依赖：CMake 3.16+、C++17、Python 3.8+、Pillow、官方 SDK2（本机 `/home/unitree/unitree_sdk2-2.0.2`）。从 pingandog 根目录：

```bash
python3 ELF-OS/control/go2.py build
python3 ELF-OS/control/go2.py observe
python3 ELF-OS/control/go2.py state
```

可以从任何工作目录用脚本绝对路径运行。`--sdk-root` 指定 SDK；构建与运行必须一致。`--build-dir`、`--runs`、`--iface` 可配置。旧 `sdk_probe.py image|state` 保留兼容入口，image 现在返回完整 Observation。

`observe` 返回 JPEG 路径、同名 `.observation.json` 和状态；默认每次生成独立文件，避免覆盖。`--output` 可指定图像路径。图像验证使用 Pillow，返回主机采集时间、原图尺寸、哈希、状态和日志路径；没有相机曝光时间和深度，不假装是同步 RGB-D。

## 运动接口

```bash
# 默认 dry-run，完全不启动 DDS 或运动 RPC
python3 ELF-OS/control/go2.py step --vx 0.2 --seconds 1.25

# 真正执行：仅在任务授权、刚检查过实时画面和站姿后使用
# observation 路径必须替换成 observe 返回的实际文件
python3 ELF-OS/control/go2.py step --vx 0.2 --seconds 1.25 --observation /absolute/path/to/frame.observation.json --execute

# 转向示例（dry-run；转向尚未实机验收）
python3 ELF-OS/control/go2.py step --vx 0 --yaw-rate 0.35 --seconds 0.5

# 独立的真实停止接口，无需 observation 或 --execute
python3 ELF-OS/control/go2.py stop

# 离散动作（默认 dry-run；真实执行需同时提供新鲜 observation）
python3 ELF-OS/control/go2.py discrete-action --action forward --value 25 --unit cm
```

- 一次只允许前进或原地转向；无侧移、后退、姿态切换。
- `vx <= 0.20 m/s`，`|yaw_rate| <= 0.35 rad/s`，`duration <= 1.25 s`；拒绝负时间、NaN/Inf 和混合运动。
- 执行前检查同 boot、同接口、60 秒内的图像记录和哈希。**这不代替人/模型的画面审查**，也不证明相机曝光时间。
- C++ 在硬件侧再次限幅，等候新鲜状态、要求初始 mode 1；移动中接受 mode 1/3。除精确值 2 外的非零 error_code、姿态超过约 20°、状态超过 0.5 秒未更新会退出并尝试 StopMove。
- Move RPC 超时 0.3 秒；循环约每 50 ms 检查，RPC 阻塞和调度会增加停止延迟。StopMove 最多尝试三次。不能保证断网/断电时停止成功。
- 单用户本地锁限制同入口并行 step；独立 stop 留取消标记供活动 step 检查并发 StopMove。其他 SDK 程序/遥控器不受该锁控制。stop 与正在进行的 RPC 有调度窗口，不是硬实时急停。
- Python 收到中断/超时会终止子进程并独立尝试停止；SIGKILL、系统崩溃没有软件保证。遥控接管与硬件 watchdog 尚需后续集成。
- `vx*duration` 为名义距离；使用状态/画面核验实际位移，不自动补走。
- `discrete-action` 只接受 forward 25/50/75 cm、turn_left/right 15/30/45 degree
  和 stop。真实 step 会在执行前查询并开启 Go2 避障服务，动作结束时发送零速度并
  释放 API 控制源；避障服务查询或控制源获取失败时拒绝运动。

## 日志及控制权

每次 SDK 调用写入 `runs/<time-uuid>/`：request.json、stdout.jsonl、stderr.log、result.json；step 关联 observation。失败不自动重试移动；只重试停止。返回码成功不等于完成指定米数，更不证明路径安全。

### 错误码排查

| 日志字段 | 来源 | 含义 |
| --- | --- | --- |
| `state.error_code` | `rt/sportmodestate` 的 `SportModeState.error_code` | 机器人广播的状态码；码 2 的固件定义尚未核实 |
| `move/stop.code` | `Move/StopMove` RPC 返回值 | 0 表示接口调用成功；不能用它解释机器人状态码 |
| `result.json.returncode` | C++ 执行器进程 | 当前受控失败退出 1，与机器人码值无对应关系 |
| CLI 退出 2 且有 `usage:` | Python 参数解析 | 参数/子命令错误，尚未调用 SDK；先检查完整命令 |

新执行器输出 `guard_rejected`，包含 `phase`、`reason`、`robot_error_code`、`mode`；前一条 `state` 与 stderr 保留状态年龄及姿态。原因可为 `state_stale`、`robot_error_code_nonzero`、`mode_not_allowed`、`posture_nonfinite`、`excessive_tilt`。最终状态检查失败也会记录原因。`result.json` 分开汇总 `guard_rejections` 和 `rpc_failures`，不再只笼统报告 SDK 失败。

2026-09-16 排查证据：旧 `captures/step_verification/state_during.log` 的样本 400–1100 为 mode 3、error 2，1200 为 mode 1、error 2，1300 起恢复 0。旧 helper 未检查该字段；新 `cpp/guard.hpp` 要求 error 为 0，所以同样状态会被新执行器拒绝。检查时已有五组新调用日志均是成功的 image/state，未发现新执行器的 step 失败记录；不能据此宣称已复现反复失败。

官方 [SportModeState 消息定义](https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg) 仅声明 `uint32 error_code`，未给出码表；[sport_error.hpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_error.hpp) 中的 4101/4201/4205 是另一组接口错误，不能解释这里的 2。本次访问官方 sports_services 文档返回 HTTP 567，未取得可验证的固件码表。2026-09-16 用户随后明确授权将精确值 2 作为警告继续执行（不是位掩码放行）；其他非零状态仍停止。后续应按机器固件版本向宇树核实该字段的定义，再决定是否需要分类处理。

生产的连续导航应建立统一控制权仲裁（Nav2/短步/遥控），并实时接入点云及落差感知。当前 CLI 不是避障控制器，不启动 Nav2，不作为公共网络服务开放。

## 网络和 DDS

已验证 NX eth0 为 `192.168.123.18/24`，连接 Go2 内部网络，SDK domain 0。wlan0 可保持外网。GO2-NX 是 NX 的热点配置，**不是 Go2 内部控制网络的证据**；不需要开热点或更改 Wi-Fi。

每个 SDK 子进程采用最小环境，优先加载 SDK 自带的对应架构 DDS 库，避免 ROS Foxy 同名旧 libddsc 的 `ddsi_sertype_v0` 冲突。不全局更改环境。

## 测试与验证范围

```bash
python3 -m unittest discover -s ELF-OS/control/tests -v
cd ELF-OS/control/build
ctest --output-on-failure
```

本轮新程序：已独立编译、通过参数/时效/错误状态等离线测试，并实际获得 RGB 和运动状态；没有为打包代码重复移动机器人。测试不证明避障、网络故障停车或转向性能。

此前真实一次短步由旧 helper 完成：名义 25 cm，位置估计从 (-2.053,1.208) 到 (-1.744,1.205)，约 0.31 m；两次停止 RPC 成功。证据保留于 `captures/step_verification/`。运动中短暂 error_code=2，随后归零；含义待核实。最初版本遇到码 2 会提前停止；现已按用户授权放宽为警告，尚未实机验证修改后的短步，因此**不能把旧测试当作新运动实现已完成实机验收**。

## 代码与 skills

- `cpp/elf_go2.cpp`：SDK 采集/状态/受限执行。
- `cpp/guard.hpp`：可离线验证的执行条件。
- `go2.py`：命令、构建、图像记录、日志和干净 SDK 子进程。
- [elf-go2-control skill](../skills/elf-go2-control/SKILL.md)：Codex 如何观察、执行并核验。
- [框图对应的架构和阶段](../docs/navigation-architecture.md)。
- [数据契约](../docs/navigation-contracts.md)。

### 本机码 2 的部署例外

用户明确授权忽略已复现的 `error_code=2`。执行器允许 0 和精确值 2，后者输出 `guard_warning`（`operator_allowed_code_2`），并汇总到 `result.json.guard_warnings`。3、6 等其他值仍拒绝；码 2 不能绕过模式、姿态或状态时效检查。此例外适用于起步、运动中和停止后状态核验，不改变 Move/StopMove 返回值检查。官方语义仍未核实。

参考链路为 `src/navila_bridge/protected_motion_executor.py` → `src/go2_comm_tests/go2_protected_motion.cpp`：旧 helper 每 100 ms 发 Move，到时 StopMove，只检查 RPC 返回值，完全不订阅机器人状态。ELF-OS 保留自己的状态保护与速度/时长限制。顶层 `navila_bridge/control_server.py` 是不执行硬件动作的早期版本。
