# 当前硬件与通信状态

最后更新：2026-05-12

## 调试环境

- 主机：Windows 笔记本
- 编辑器：VSCode + Codex
- 低层通信环境：conda `soarm101`
- LeRobot 环境：conda `lerobot`
- IK 环境：conda `soarm101_ik`
- 低层串口依赖：`pyserial`

## 通信硬件

- 驱动板：Waveshare Bus Servo Adapter (A)
- 驱动板模式：USB 控制模式，跳线帽在 B 位置
- 当前 Windows 串口：`COM5`
- 舵机波特率：`1000000`
- 舵机型号：STS3215
- ping 读到的型号编号：`777`
- 供电：舵机必须使用外部电源，USB 只负责串口通信

## 舵机状态

| ID | 关节名 | 中文含义 | 当前状态 |
| --- | --- | --- | --- |
| 1 | `shoulder_pan` | 底座旋转 | 已标注、已装机、通信通过 |
| 2 | `shoulder_lift` | 肩关节抬升 | 已标注、已装机、通信通过 |
| 3 | `elbow_flex` | 肘关节弯曲 | 已标注、已装机、通信通过 |
| 4 | `wrist_flex` | 腕部俯仰 | 已标注、已装机、通信通过 |
| 5 | `wrist_roll` | 腕部旋转 | 已标注、已装机、有物理限位，需要软件限幅 |
| 6 | `gripper` | 夹爪 | 已标注、已装机、通信通过 |

## 已完成事项

- Windows 可识别微雪驱动板串口。
- STS3215 单舵机 ping 成功。
- 六个舵机 ID 已完成标注。
- 六个舵机总线扫描通过。
- LeRobot calibration 文件已包含 6 个关节。
- `robot.send_action()` 小幅动作验证通过。
- URDF + IK 离线求解链路已建立。

## 重要注意

- ID5 `wrist_roll` 的机械结构存在物理限位，不应按无限旋转舵机处理。
- 重新标定或设置 ID5 中位前，先备份 LeRobot calibration 文件。
- 大动作必须先 dry-run，再确认目标角度和机械空间。
- 如果出现卡滞、撞限位、异常持续旋转，优先断电；能通信时再执行 `torque_off.py`。
