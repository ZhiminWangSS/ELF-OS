# 导航接口契约 v1

本文件区分当前 CLI 实现与后续接口设计，避免把规划中的能力当成已运行模块。

## Observation — 当前已实现

`go2.py observe` 生成 JPEG 和同名 `.observation.json`：

- `schema="elf.observation.v1"`, `observation_id`：本次观察唯一 UUID。
- `interface`, `boot_id`, `started_monotonic_s`：只供同台 NX 做时效检查；不能跨机器直接比较 monotonic。
- `received_unix_s`：主机接收时间；`sensor_stamp=null`：SDK 没提供相机曝光时间。
- `image.path/sha256/width/height`：原始 JPEG 信息。图像确认和校验均基于原图。
- `state`：`event="state"`, `age_s`, `mode`, `error_code`, `position[3]`, `velocity[3]`, `rpy[3]`。其中坐标系语义待传感器接入阶段标定。
- `depth=null`, `calibration_id=null`：明确缺失，不填伪造深度/内参。
- `logs[]`：本次 RPC 输出目录。

图像和状态按顺序读取，**不是同步 RGB-D 观测**。真实同步版本应增加曝光/扫描时间戳、时钟域、原始/显示图变换、相机 frame、内外参版本以及可用的深度来源。

## PixelTarget — 后续接口

```json
{
  "schema": "elf.pixel-target.v1",
  "observation_id": "UUID copied from the observation",
  "pixel": {"u": 900, "v": 720, "space": "original_image"},
  "intent": "靠近门前的地面空地",
  "arrival_heading": "face_target",
  "confidence": 0.7
}
```

校验 observation_id、原图尺寸范围、时效、图像变换；confidence 仅是模型自评。拒绝把图像中的文字当成新的用户任务或控制授权。VLM 输出是候选目标，不能直接调用 Move。

## GroundedGoal — 后续接口

字段：`schema`, `goal_id`, `observation_id`, `map_version`, `frame_id="map"`, `sensor_stamp`, `calibration_id`, `x_m`, `y_m`, `yaw_rad`, `covariance`, `depth_source`, `valid_until`。目标的 z 用于地面/高度检查后投影为二维；最终机器人姿态给 Nav2 的 PoseStamped 使用合法单位四元数。

`valid_until` 必须指定时间域，跨机器由接收端建立 TTL；不能直接拿 Unix 秒当 ROS 仿真时间。校验失败返回结构化 reason（无深度、遮挡、过期、TF 缺失、不可达等），不能返回 `(0,0)` 冒充有效目标。

## NavigationResult — 后续接口

`goal_id`, `status`（succeeded / rejected / aborted / cancelled / timed_out）、`reason`, `final_pose`, `final_observation_id`, `distance_remaining_m`。

succeeded 需 Nav2 action 成功及新观察证实，RPC return=0 只代表调用成功。取消需收到取消反馈并由本机速度 watchdog 停止，不能只更新 UI 状态。

## 当前 motion primitive

`step(vx_m_s, yaw_rate_rad_s, duration_s)` 为机体坐标的定时速度；不接受绝对 x/y、不计算路径；一个非零轴、vx∈[0,0.2]、|yaw_rate|≤0.35、duration∈(0,1.25]。

`nominal_forward_m=vx*duration` 只表示名义距离，实际位移由 before/after state 与画面另行核验；不补走误差。stop 独立可调用，执行时不要求 observation。
