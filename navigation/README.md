# 一层楼导航准备与短距离测试

当前地图：`maps/floor_1789552084236/`。本目录用于 **PCD 先验定位 + Nav2 + 独立 SDK 速度适配**，不替换原始建图文件。

## 当前状态（2026-09-16）

软件已构建并完成离线联调，**尚未达到真机行走放行条件**：

1. `config/robot.yaml` 的机身占地、雷达安装位置/姿态和碰撞高度是待测值，`verified: false`。需测量包括机械臂等突出部分的实际尺寸，并对比实时点云与地图后确认。
2. 实时雷达不能通过默认建图起点或最后关键帧位置的定位检查。需要确认现场当前位置/朝向，并给出正确的初始位置。没有降低 ICP 阈值来强行放行。
3. SDK 只读检查能读取新鲜机器人状态（mode=1，error_code=0），但没有收到 `rt/wirelesscontroller` 遥控数据。需开机连接遥控器，或查明该固件的真实遥控数据接口后验证接管。

以上均由 `tools/preflight.py` 检查；未通过时 `tools/hardware_test.py --execute` 拒绝启动运动接口。尺寸的现场真实性不能由软件自动证明。最后一次检查见 `log/preflight.json`。

## 地图处理

原始 `map.pcd` 保存于 `../mapping/fastlio_ws/maps/floor_1789552084236/`，包含 207 个关键帧、9 次接受的回环；这不等于覆盖整层楼或已验证精度。

保存坐标系相对地面约倾斜 28.66°。`tools/prepare_map.py` 用轨迹估计初始法向、对地面点稳健拟合，输出：

- `map.pcd`：调平并把地面移到 Z=0 的先验地图。
- `map.pgm` + `map.yaml`：5 cm 分辨率二维候选地图。黑=障碍，白=测量射线支持的空闲，灰=未知。
- `manifest.json`：原图哈希、`map_from_saved` 变换、地面残差、切片高度等。
- `preview.png`：二维地图叠加蓝色建图轨迹；`trajectory_xyz.csv` 是调平后的轨迹。

只沿保存关键帧中确有返回的有限射线清空网格，已知障碍阻挡后方清空，未知空间保留。当前高度带为离地 0.15–1.2 m，需要与实际整机高度/低障碍风险匹配。不得通过涂白墙壁、删除障碍、允许未知空间来解决规划失败。地面以下的落差、台阶、玻璃和细线缆仍需现场排除。

地图预览显示环形走廊及右侧大厅；房间与边缘大量区域仍未知。首次测试应在已观测且足够宽的局部区域。`log/map_audit.json` 的边界约 53×53.5 m 是点云外接范围，不是已覆盖面积；实测射线支持的自由格约 396 m²。不能宣称已经完成整层楼的导航覆盖。

重新转换（必须使用新的输出目录，避免覆盖已有结果）：

```bash
python3 tools/prepare_map.py ../mapping/fastlio_ws/maps/floor_1789552084236 maps/new_candidate
```

若更换地图，需要同步定位配置、Nav2 配置和启动文件中的地图路径；工具当前固定使用上述这一份地图。

## 构建与只读预览

在本目录执行：

```bash
bash build.sh
source env.bash
bash start_preview.sh rviz:=true
```

雷达驱动需要已启动。本脚本使用 ROS domain 42、localhost 和 CycloneDDS，Wi-Fi 外网及机器人 SDK domain 0 保持原拓扑。`config/cyclonedds.xml` 将自动参与者索引上限增至 100，避免 Foxy 默认 10 个参与者不足导致节点启动失败。不要同时启动原建图 FAST-LIO/回环后端或旧导航控制程序。

预览只发布 `/navigation/cmd_vel` 和 `/navigation/guarded_cmd_vel`，**不启动 SDK 运动接口**。RViz 提供地图、实时障碍、路径和坐标系显示，工具栏只提供初始位置，不提供直接发送任意导航目标的工具。

机器狗原地站稳后，在 RViz 中用 **2D Pose Estimate** 指定位置和朝向；也可用：

```bash
python3 tools/set_initial_pose.py --x <地图X米> --y <地图Y米> --yaw-deg <朝向度数>
```

输入是调平地图中的机身地面投影位置，程序转换为 IMU 位姿给初始匹配器。需要接近真实位置，不能在重复走廊随意点选。已初始化后禁止在线跳变重置；先停止导航，再重启预览。

匹配器要求连续 5 次一致结果、至少 65% 点落在 0.3 m 对应范围内、对应点 RMSE < 0.15 m，以及相对初始猜测的位移/转角限制。这不是全局定位器。

定位前端持续使用固定先验地图，不再增量建图。坐标适配器输出水平 `map -> odom -> base_link -> lidar_imu`；`map == odom` 对应当前固定先验地图模式，不另外运行 AMCL。原 LIO 的倾斜局部位姿仅通过 `/lio/odom` 传递，不重复广播 TF。安装变换用于恢复机身中心，姿态变化仍保留在动态雷达变换中。

## 安装尺寸与配置

测量并更新 `config/robot.yaml`：

- `base_from_imu_xyz`：机身中心地面投影到雷达 IMU 的位置，前/左/上，单位米。
- `base_from_imu_rpy`：雷达 IMU 原生坐标系到机身坐标系的旋转，弧度；不是驱动旋转后的点云坐标系。
- `footprint`：整机投影外包络，应包含机械臂、载荷等突出部分。
- 障碍高度带需要覆盖实际碰撞空间，与二维地图的生成高度带一致。

量值与现场点云对齐核验完成后，才能将 `verified` 改为 `true`。启动预览时会同步 footprint 至两个 costmap；如高度带改变，脚本会要求重新生成并审核地图。

雷达驱动当前把点云绕 Y 轴旋转 +30°，IMU 保留原生方向。定位前端与 ICP 都使用 -30° 撤销该点云旋转，不能再次叠加旧文档的 +30° 外参。

## 放行检查与真机测试流程

```bash
source env.bash
python3 tools/preflight.py
```

检查不发任何运动命令。应当看到 `ready_for_controlled_motion_test: true`；它要求安装尺寸已确认、定位持续健康、上游没有非零速度、新鲜机器人健康站立状态，以及新鲜且未操作的遥控数据。该检查不替代现场确认地面、通道和人员情况。

在放行后先只规划路径：

```bash
python3 tools/goal.py --x <目标X> --y <目标Y> --yaw-deg <目标朝向>
```

确认机器人位置、实时墙体与地图吻合，路线绕开墙壁，整机足迹通过门口且不碰撞。首次目标建议 0.5 m，清空周围区域，现场人员握住遥控器；禁止首轮直接跨楼层或整层楼自动运行。

**以下命令仅用于现场准备完成后主动开始测试，本轮未执行：**

终端 A（一次接口会话最多 30 秒，允许参数最大 60 秒）：

```bash
python3 tools/hardware_test.py --execute --seconds 30
```

终端 B（工具限制目标距当前位置不超过 1 m）：

```bash
python3 tools/goal.py --x <目标X> --y <目标Y> --yaw-deg <目标朝向> --navigate --timeout 15
```

速度硬上限为前进 0.20 m/s、转向 0.35 rad/s，不允许倒退或侧移。初次到达/失败/取消后，目标工具会请求本机运动接口停止；下一次测试需重新通过检查并启动接口。

停止方式：

```bash
python3 tools/goal.py --cancel
# 独立停止入口，导航本身异常时也可使用：
python3 /home/unitree/pingandog/ELF-OS/control/go2.py stop
```

终端 A 的 Ctrl+C、会话时限到期也会关闭接口并请求停止。遥控器任意按键或摇杆输入会使 SDK 接口停止并退出，交还控制；需要实机验证遥控数据与实际接管行为。

## 速度与失联保护

ROS 与 Unitree SDK 分进程运行，避免 DDS 动态库冲突。通过本机 Unix socket 传递有序速度指令，保持原始接收时间，不能靠重发心跳延长旧速度寿命。

SDK 进程独立验证 250 ms 指令超时、限速、有限数值、单调序号、机器人状态、遥控新鲜度及接管。本机固件在接受 `Move` 后会持续报告精确值 `error_code=2`；按基础执行器已有的现场授权将其记录为警告并继续，其他非零错误码仍拒绝。运动中失去上游健康状态会停止并锁住，恢复消息不会自行恢复行走。软件循环/SDK RPC 有调度与调用时延；不是硬件独立急停，也不能把 RPC 成功当作物理静止证据。

接口复用 `/tmp/elf-go2-<uid>/motion.lock` 和 `cancel`，与现有 ELF 短步控制互斥。不能仲裁未遵守此锁的其他 SDK 客户端，因此实际测试前必须关闭旧 `sport_mode_ctrl` 等控制程序。

Nav2 配置禁止未知区域规划，使用整机足迹碰撞检查，取消自动旋转/后退脱困。实时点云提供动态障碍，定位匹配比例、残差、时间戳、点云和位姿新鲜度共同决定速度放行。

## 已完成验证与证据

| 验证 | 结果与证据 |
|---|---|
| 地图解析、倾斜地面恢复、射线穿墙防护 | `tests/test_map.py`，3 项通过 |
| 速度限幅、取消归零、失联、非法输入锁住 | `tests/test_velocity_guard.py`，4 项通过 |
| 已保存关键帧 ICP | `log/initializer_smoke.json`：位置误差约 7.9 mm、角度误差约 0.0045 rad；远离地图的初值拒绝 |
| 固定先验 LIO + 坐标/点云适配 | `log/localization_smoke.json`：162 帧位姿、静止 XY 最大误差约 8.2 mm；传感器中断后健康状态失效；IMU 为合成输入 |
| 安装的 Foxy Nav2 在本地图上规划/控制/取消 | `log/nav2_smoke.json`：153 个路径点，未进入占据/未知格，中心线最小净空约 0.716 m；占据/未知目标拒绝；取消后限速输出为零 |
| 编译后的 SDK worker mock | `log/sdk_smoke.json`：指令超时停止且不自动恢复，NaN 和超速拒绝；未初始化硬件 DDS |
| SDK 状态/遥控策略 | `sdk/policy_test.cpp` 与 `log/sdk_policy_test.log` |
| 实时只读检查 | `log/preflight.json`；机器人状态正常，但遥控数据缺失，定位和尺寸检查未通过 |

离线测试不会证明机器人实际运动、避障、刹停距离或遥控接管。之后依次实测：短程到达、取消停止、遥控接管、低速前方障碍、定位/雷达中断停止。完成后再扩大到多段走廊和房间。

## 回归测试

Python 单元测试不依赖 ROS：

```bash
python3 -m unittest discover -s tests -v
```

ROS 联调必须隔离通信域，不能在机器人 domain 42 注入模拟位姿：

```bash
source env.bash
ROS_DOMAIN_ID=44 ros2 launch "$PWD/launch/navigation.launch.py" localization:=false
# 另一个已 source env.bash 的终端：
ROS_DOMAIN_ID=44 python3 tests/nav2_smoke.py
# 测试完停止上面的 launch
ROS_DOMAIN_ID=45 python3 tests/initializer_smoke.py
ROS_DOMAIN_ID=46 python3 tests/localization_smoke.py
python3 tests/sdk_smoke.py
(cd sdk/build && ctest --output-on-failure)
```
