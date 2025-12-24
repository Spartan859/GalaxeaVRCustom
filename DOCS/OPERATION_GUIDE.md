# Galaxea VR → R1 Pro Teleop 操作指南

## 简介
本指南介绍如何运行并测试 Unity 端的 `ControllerScript.cs`（Quest 3 手柄客户端）与 Python 端的 `teleop_vr.py`（RemoteVRTeleop TCP 服务器），以及两端使用的通信协议与故障排查要点。

## 先决条件
- Unity 项目已打开并能在 Quest 3 上运行；`ControllerScript.cs` 已挂到合适的 GameObject。
- 在工作站上有 Python 环境（建议 Python 3.8+）并能运行 `teleop_vr.py`。
- 网络连通：Unity 设备能访问运行 `teleop_vr.py` 的主机/端口。

## 文件位置（仓库）
- Unity 客户端: Assets/Scripts/ControllerScript.cs
- Python 服务: teleop_vr.py
- 本指南: DOCS/OPERATION_GUIDE.md

## 协议说明（重要）
- 传输: TCP 长连接
- 编码: 消息以 UTF-8 JSON 序列化
- 帧格式: 4 字节大端（big-endian）长度前缀（uint32）后跟消息主体

示例消息：
```json
{"cmd":"send_action","action":{...}}
```
常见回复：
- action 响应: `{"cmd":"action","ok":true}`
- pong 响应: `{"cmd":"pong","ok":true}`

## Unity 端 (ControllerScript.cs) 配置要点
- 在 Inspector 中设置 `serverHost` 和 `serverPort`（与运行 Python 服务器的主机/端口匹配）。
- `sendInterval` 控制发送频率（默认 0.1s）。`pingInterval` 控制心跳。
- `maxQueueSize` 控制发送/接收队列最大长度，超过则丢弃最老消息以防内存堆积。
- `joystickDeadzone` 避免小幅抖动被发送。

运行逻辑要点：
- 机械臂姿态与夹爪采用“差量”发送（每帧发送增量），在首次发送会发送零增量以建立基准。
- 底盘（`chassis_speed`）和躯干（`torso_speed`）代表持续速度，不是差量。
- Reset 通过 `send_action` 的 `action.reset` 字段（true/false）传递。
- 出现 action 或 ping 响应超时时，客户端会尝试重连（有限次），并带有防止并发重连与最终停止的保护（`isReconnecting`、`connectionFailed`）。

## Python 端 (RemoteVRTeleop / teleop_vr.py) 要点
- 启动服务器：
```bash
python teleop_vr.py
```
默认监听 `0.0.0.0:50051`（可在构造器中修改）。
- 支持命令：`send_action` 和 `ping`。
- `get_action()` 返回与 Unity 完全一致的 action 结构：
  - `left_ee_pose`: 6 floats (dx,dy,dz,droll,dpitch,dyaw)
  - `right_ee_pose`: 6 floats
  - `left_gripper`, `right_gripper`: float（差量）
  - `chassis_speed`: 3 floats（vx, vy, w） — 持续速度
  - `torso_speed`: 4 floats（vx, vz, w_pitch, w_yaw） — 持续速度
  - `reset`: bool
- 因增量字段的语义，`get_action()` 在返回后会把增量字段（`left_ee_pose`、`right_ee_pose`、`left_gripper`、`right_gripper`、`reset`）清零，以避免重复应用相同增量；但会保留速度字段（`chassis_speed`、`torso_speed`）。

## 启动顺序建议
1. 在 PC（或服务器）上启动 `teleop_vr.py`。确认监听端口无误并输出等待连接日志。
2. 启动 Unity 并在 Inspector 设置 `ControllerScript` 的 `serverHost` 指向运行 `teleop_vr.py` 的主机（如局域网 IP 或电脑名）。
3. 在 Unity 中运行场景，观察 Unity 控制台和 Python 端日志的连接与心跳信息。

## 测试与验证
- 连接验证：Unity 启动后应看到“已连接到服务器”日志；Python 端应显示客户端已连接。
- Ping 测试：每秒发送心跳，若无 pong 或 pong 返回异常，将触发重连逻辑并在日志中记录。
- 发送动作：在 BiManual 模式下移动手柄会发送 `send_action`；在 `teleop_vr.py` 通过 `get_action()` 获取同样的结构并应用到机器人控制逻辑。
- Reset 测试：在 Unity 触发 reset（右摇杆长按）时，`action.reset` 字段应为 `true`，Python 端应在下一次 `get_action()` 返回中看到 `reset: true`（且随后被清零）。

## 故障排查（常见问题）
- 问：频繁看到“action响应超时”或“ping响应超时”。
  - 检查 Python 服务器是否仍在运行并监听正确端口。
  - 检查网络连通性（PC 与 Quest 是否在同一子网／路由阻止了流量）。
  - 在 Python 端查看是否有异常堆栈或进程被阻塞。

- 问：客户端一直在重复尝试重连（每秒/每两秒打印“尝试重连…”）。
  - 现在实现了 `isReconnecting` 与 `connectionFailed` 保护：首次触发重连后会进行有限次重试；失败达到上限后会停止进一步尝试并停止发送。
  - 若仍看到重复尝试，请确认是否有其它代码调用 `ReconnectAsync()` 或检查日志以定位触发点。

- 问：动作（增量）重复应用。
  - 确认 Python `get_action()` 行为：增量字段在读取后会被清零，若仍重复，可能是 Unity 端多次发送相同增量（检查 Unity 中的 `hasLastSend` 与 `last*` 基准记录）。

## 日志与调试要点
- Unity 日志位置：Unity Editor Console 或设备日志（adb logcat for Quest）。
- 关注关键日志：连接建立/断开、action 响应超时、ping 响应超时、重连次数到达上限。

## 推荐的进一步改进（可选）
- 在 Python 端添加基于时间的连接超时检测，自动清理僵尸连接。
- 在 Unity 端暴露一个 UI 按钮用于手动重连/退出，便于调试。
- 将协议版本号加入首条消息，便于向后兼容扩展字段。

---
如果你希望，我可以：
- 把这份指南扩展为更详细的步骤截图版；
- 在 README 中加入启动脚本与一键运行示例；
- 或者把需要的调试命令（adb、netstat 等）加入页面。

已在仓库创建： `DOCS/OPERATION_GUIDE.md`。