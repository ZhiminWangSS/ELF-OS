# 第一步：启动 MID360 雷达（Ubuntu 20.04 / ROS 2 Foxy）

工作目录：`/home/unitree/pingandog/ELF-OS/mapping/ws_livox`。
此步骤只启动雷达并检查数据，不启动 FAST-LIO、不控制机器狗运动。

## 已确认的问题和修复

- 本机已有 Livox-SDK2 头文件、动态库和可用的驱动源码；无需重复下载。
- `eth0` 没有 IPv4 地址时，SDK 无法绑定配置中的 `192.168.123.18`，报 `bind failed`。仅编译成功不能说明雷达可用。
- 现有 `Wired connection 1` 已配置 `192.168.123.18/24`，没有网关且禁止成为默认路由；启用它即可恢复雷达网段。
- 原驱动在无数据时退出，会等待未唤醒的数据线程。修复为退出前唤醒点云及 IMU 队列。
- ROS 2 包声明补充 `builtin_interfaces`、`ament_index_python`、`launch`、`launch_ros`。
- 新增本机 IP/端口启动检查、固定工作目录的编译入口，以及实际订阅点云和 IMU 的验收工具。

2026-09-16：用户已启用有线连接，雷达 ping 通，驱动初始化成功。默认 Fast DDS 环境下出现 `bad_alloc`，随后驱动内存增长到约 11 GB，被系统 OOM 终止；仅设置 `ROS_LOCALHOST_ONLY=1` 仍复现。当前使用已安装的 CycloneDDS、独立域 42 和本机通信，持续运行验收记录保存在 `log/step1_acceptance.log` 与 `log/step1_memory.log`。这是一组经实机验证的规避配置，尚未定位 Fast DDS 内部根因。

驱动重新编译通过；缺失本机 IP 和 UDP 端口占用能明确报错；无数据退出测试约 0.27 秒完成，没有升级为强制终止（记录见 `log/step1_shutdown_check.log`）。

实机数据验收已通过：连续 180 秒收到 1,799 帧点云（10.00 Hz）和 35,963 条 IMU（200.02 Hz）；最大接收间隔分别为 0.186 秒和 0.018 秒；前、中、后三段内容抽检通过，每帧 19,968–20,256 点。无发布者的独立域负向测试能返回失败，避免把没有数据误判为通过。

内存验收：180 秒内 19 次采样，RSS 为 27,332–28,888 kB，无交换内存使用。最终检查时驱动已连续运行超过 8 分钟，仍在运行，未出现之前的 OOM。此结果证明本次持续测试通过，不等同于无限时长稳定性保证。

## 启动

在本机终端启用已有的雷达网络连接（需要管理员权限）：

```bash
sudo nmcli connection up 'Wired connection 1' ifname eth0
ip -4 addr show eth0
ping -c 3 192.168.123.3
```

本机 IP 应为 `192.168.123.18/24`，雷达配置 IP 为 `192.168.123.3`。
不要将 Wi-Fi IP 填入雷达配置。ping 只是连通性参考，最终以 ROS 消息验收为准。

启动并保留终端运行：

```bash
cd /home/unitree/pingandog/ELF-OS/mapping/ws_livox
bash start_mid360.sh
```

脚本加载 `mapping_env.bash`（ROS 2、本工作空间、CycloneDDS、域 42、本机通信），检查已安装配置中使用的本机 IP 与 UDP 端口，再执行教程的启动命令。
若提示端口占用，请检查是否已有驱动运行，勿重复启动。
若手动执行教程启动命令，应先加载新增的统一环境；此方式跳过网络预检查：

```bash
cd /home/unitree/pingandog/ELF-OS/mapping/ws_livox
source mapping_env.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 验收

在另一个终端运行：

```bash
cd /home/unitree/pingandog/ELF-OS/mapping/ws_livox
source mapping_env.bash
python3 verify_mid360.py --seconds 10
```

验收按当前 10 Hz 点云、200 Hz IMU 配置设置阈值。成功必须同时收到 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）和 `/livox/imu`（`sensor_msgs/msg/Imu`）。脚本直接统计序列化消息的接收频率，避免 Python 逐点转换拖慢计数；测量后抽检前、中、后三段各一帧，检查时间戳递增、点云非空、点数一致、坐标和 IMU 数值有限。
成功要求点云 ≥8 Hz、IMU ≥160 Hz、接收覆盖测量窗口至少 70%，没有超过 1 秒的接收中断；退出码为 0，否则为 1。默认窗口 10 秒，持续验收使用 `--seconds 180`。内容检查是抽样，不代表每个点都经过验证。

后续 FAST-LIO 必须在同一终端先 `source /home/unitree/pingandog/ELF-OS/mapping/ws_livox/mapping_env.bash`，再加载 FAST-LIO 工作空间并启动，使它使用相同通信实现和域。当前配置只在机载电脑发布 ROS 话题；外部电脑不能直接订阅。本机 Wi-Fi 默认上网路由不受影响。

## 重新编译

```bash
cd /home/unitree/pingandog/ELF-OS/mapping/ws_livox
bash build_mid360.sh
```

源码位于 `../livox_ros_driver2`，工作空间通过 `src/livox_ros_driver2` 链接它。
若迁移到新机器后链接缺失，可在上述目录运行 `ln -s ../../livox_ros_driver2 src/livox_ros_driver2`（仅在链接不存在时）。
修改源码配置 `../livox_ros_driver2/config/MID360_config.json` 后也需重新编译安装，启动读取的是 `install/` 中的配置。
不要在当前链接布局下运行上游 `build.sh`；它依赖相对路径并删除 build/install，可能操作错误目录。

## 新机器缺依赖时

当前机器无需重复安装。迁移机器时先准备 ROS 2 Foxy，再安装驱动依赖：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libapr1-dev libpcl-dev \
  python3-colcon-common-extensions ros-foxy-ament-cmake-auto \
  ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
  ros-foxy-rclcpp ros-foxy-rclcpp-components ros-foxy-rcutils \
  ros-foxy-std-msgs ros-foxy-sensor-msgs ros-foxy-builtin-interfaces \
  ros-foxy-rcl-interfaces ros-foxy-pcl-conversions ros-foxy-rosbag2 \
  ros-foxy-ament-index-python ros-foxy-launch ros-foxy-launch-ros ros-foxy-rclpy \
  ros-foxy-rmw-cyclonedds-cpp
```

Livox SDK2 官方来源：<https://github.com/Livox-SDK/Livox-SDK2>。
本项目已有副本，可从任意目录执行：

```bash
cmake -S /home/unitree/pingandog/ELF-OS/Livox-SDK2 \
  -B /home/unitree/pingandog/ELF-OS/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/unitree/pingandog/ELF-OS/Livox-SDK2/build -j2
sudo cmake --install /home/unitree/pingandog/ELF-OS/Livox-SDK2/build
sudo ldconfig
```

驱动上游：<https://github.com/Livox-SDK/livox_ros_driver2>。
当前源码的 Git remote 是教程维护者的 Gitee 副本；本次保留已有代码及设备配置，没有直接覆盖为上游最新版本。
