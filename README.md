# Galaxea VR → R1 Pro Teleop 操作指南

## 键位说明

- 底盘：左摇杆前后/左右控制底盘 vx/vy；右摇杆左右控制底盘角速度 w；按下右摇杆后，右摇杆左右改为躯干偏航。
- 躯干：右摇杆上下控制躯干升降 vz；A/B 控制躯干前后平移（A 向后、B 向前）；X/Y 控制躯干俯仰角速度（X 俯、Y 仰）。
- 手臂发送：Normal 模式下需按住对应握把才发送该侧增量，松开则该手臂增量置零；Event 模式下两侧增量置零，仅发送事件/干预标志。
- 夹爪：按住左/右食指扳机持续收紧；短按（时间 < gripperShortPressTime）快速回到 100% 张开。
- 上层模式：左右摇杆按钮同时按下后松开切换 Normal/Event（组合的下降沿），切换会重置增量基准。
- 事件（仅 Event 模式）：A=success，B=rerecord，X=terminate（按键下降沿生效）。
- 重连：当 connectionFailed 时同时按下 A+X 触发一次重连并清零重连计数。
