# Go2 SportModeState.error_code=2 核查

日期：2026-09-16。结论：已复现实机状态码 2 触发本地停止；尚未取得对应固件的码值定义，不能认定正常或指定故障。

## 实机证据

- SDK 目录：`/home/unitree/unitree_sdk2-2.0.2`。
- 只读调用 `SportClient.GetApiVersion()` 和 `GetServerApiVersion()`：均返回 `1.0.0.1`。这是 API 版本，不是整机固件版本；未查到可用的本机整机固件记录。
- 真实短步日志：[stdout.jsonl](runs/20260916T183548-b426ebbfc82645dca5056a7a0576e2e1/stdout.jsonl)。请求前进 0.1 m/s、0.5 s；初始 mode 1/error 0；一次 Move 返回 0；下一次检查 mode 3/error 2，触发 `robot_error_code_nonzero`；StopMove 返回 0；最终 mode 1/error 0。用户观察到确实动了一下。
- 日志能证明本地 guard 是动作提前终止的直接原因，不能单凭时间上的关联解释码 2 的语义。
- 本轮核查只读取版本与外部资料，没有发送移动或姿态指令，没有改变检查规则。

## 资料核对

1. [宇树官方 ROS 2 README](https://github.com/unitreerobotics/unitree_ros2#1-sportmode-state)：`error_code` 只注释为 `Error code`，无码表；另一个字段 `mode` 列出了 1=balanceStand、2=pose、3=locomotion、6=jointLock、7=damping。不能把 mode 的 2=pose 套用到 error_code。
2. 官方本机 SDK 的 `SportModeState_.hpp` 只定义 uint32 字段；`sport_error.hpp` 的 4101/4201/4205 属于另一组接口错误，并非状态码 2 的解释。
3. 官方文档站 `Basic_services`、`sports_services` 本次均返回 HTTP 567，无法读取内容。公共文本代理也连接超时；不能声称已读到码表。
4. [官方 Python SDK issue #175](https://github.com/unitreerobotics/unitree_sdk2_python/issues/175) 有开发者询问同一字段的含义，报告 1001/1002。检查到的回复来自独立集成者（author_association=NONE），明确表示未找到枚举，不是宇树官方解释，也不对应本机的 2。
5. [第三方 V2.0 实机项目](https://github.com/Oya-Tomo/unitree-go2-zenoh-node/blob/main/docs/adr/0001-observed-state-driven-control-architecture.md) 将该字段用于运动状态机 ID（如 100/1001/1002）。这提示字段语义可能依版本不同，但不能据此把本机的 2 白名单化。

## 待核实的具体问题

需要整机软件/运动控制固件版本对应的官方 `SportModeState.error_code` 定义：它是枚举还是位掩码？十进制 2（0x00000002）具体表示什么？行走时出现、停止后消失是否为预期？此状态下 Move 是否应继续？API 版本 1.0.0.1 本身不足以回答这些问题。可从宇树 App 版本页面取得整机版本，再与宇树对应版本文档或技术支持核对。

## 后续用户授权

用户随后明确要求忽略码 2。执行策略现已将精确值 2 作为警告继续，其他保护保持。此变更是本机部署选择，不是官方语义已获核实。
