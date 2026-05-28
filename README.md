# SO-ARM101 / SO101 机械臂视觉打靶项目

本仓库是 SO-ARM101 / SO101 机械臂项目的 Windows 调试与后续集成工作区。当前目标不是直接上车跑 ROS，而是先把“硬件通信可靠、舵机 ID 与标定正确、LeRobot 能发送动作、URDF + IK 能求解”这条主线整理清楚，为后续接入 autolabor2.5 底盘、YOLOv8 视觉识别和 ROS 节点做准备。

## 当前硬件

- 机械臂：SO-ARM101 / SO101，6 个 STS3215 总线舵机
- 舵机驱动板：Waveshare Bus Servo Adapter (A)
- 调试主机：Windows 笔记本 + VSCode
- 当前串口：`COM5`
- 舵机波特率：`1000000`
- 供电原则：USB 只负责串口通信，舵机必须接外部电源

## 项目结构

```text
assets/urdf/                  SO101 URDF 模型
config/                       舵机 ID、关节映射等共享配置
docs/                         中文流程、硬件状态、标定和 ROS/IK 说明
tools/stservo/                微雪 STServo 低层通信与排障工具
tools/lerobot/                LeRobot 读取状态、发送动作、ID5 中位设置工具
tools/ik/                     URDF + IK 离线求解工具
vendor/waveshare_stservo/     微雪官方 STservo SDK，仅保留通信库
requirements.txt              低层 STServo 工具的最小依赖
```

详细说明见 [docs/project-structure.md](docs/project-structure.md)。

## 舵机 ID 映射

| ID | LeRobot 关节名 | 中文含义 |
| --- | --- | --- |
| 1 | `shoulder_pan` | 底座旋转 |
| 2 | `shoulder_lift` | 肩关节抬升 |
| 3 | `elbow_flex` | 肘关节弯曲 |
| 4 | `wrist_flex` | 腕部俯仰 |
| 5 | `wrist_roll` | 腕部旋转 |
| 6 | `gripper` | 夹爪 |

共享配置文件：[config/servo_map.json](config/servo_map.json)。

## 常用命令

低层 STServo 通信检查使用 `soarm101` 环境：

```powershell
conda activate soarm101
python -m pip install -r requirements.txt
python .\tools\stservo\scan_ids.py --port COM5
python .\tools\stservo\read_positions.py --port COM5 --count 1
python .\tools\stservo\servo_status.py --port COM5 --id 5
```

LeRobot 状态读取和动作发送使用 `lerobot` 环境：

```powershell
conda activate lerobot
python .\tools\lerobot\read_so101_observation.py --port COM5 --id soarm101_follower --count 1
python .\tools\lerobot\send_joint_action.py --port COM5 --id soarm101_follower --max-relative 5 --set shoulder_pan=0 shoulder_lift=0 elbow_flex=0 wrist_flex=0 wrist_roll=0 gripper=0
```

上面第二条默认是 dry-run，不会真正运动。确认目标安全后再追加 `--yes`。

末端 IK 偏移的一键调试命令：

```powershell
conda activate lerobot
python .\tools\lerobot\send_ik_delta.py --port COM5 --id soarm101_follower --dx 0.005 --dy 0 --dz 0
```

确认目标安全后再追加 `--yes`。

末端绝对坐标控制模板：

```powershell
python .\tools\arm_control\send_absolute_pose_template.py --x 0.30 --y 0 --z 0.20 --roll 0 --pitch 0 --yaw 0
```

查看当前由标定关节范围派生出来的安全工作空间：

```powershell
python .\tools\arm_control\send_absolute_pose_template.py --show-workspace
```

这个模板默认 dry-run，并通过微雪总线配置直接映射到 STS3215 原始位置。工作空间不是手写经验半径，而是从 [config/absolute_pose_control.json](config/absolute_pose_control.json) 里的标定关节范围收缩到 95% 后，经 URDF 正运动学采样得到。

地面靶位打靶动作 dry-run：

```powershell
python .\tools\arm_control\hit_target_action.py --port COM5 --x 0.25 --y 0 --z 0.02
```

打靶程序会保持 X/Y 为靶心，沿 Z 方向生成 `raise_above -> strike_down -> hit_hold -> rebound_up` 的轨迹，并在回收前检查末端禁入区。确认轨迹、IK 结果和禁区检查安全后再追加 `--yes`。动作参数在 [config/hit_action.json](config/hit_action.json) 中调整。

人工记录基座附近禁入区域：

```powershell
python .\tools\arm_control\record_forbidden_zone.py --port COM5 --id soarm101_follower
python .\tools\arm_control\build_forbidden_zone.py --type cylinder --safety-margin 0.02
python .\tools\arm_control\hit_target_action.py --port COM5 --x 0.20 --y 0 --z -0.20 --strike-height 0.12
```

`record_forbidden_zone.py` 默认只读取舵机当前位置，不发送保持指令。如需人工拖动机械臂，可先使用 `tools/stservo/torque_off.py` 释放扭矩。当前禁入区只检查“末端点”是否进入人工拟合的圆柱体或 AABB，不做夹爪外形和全连杆碰撞检测；夹爪之外的连杆仍可能碰撞。后续可接入 PyBullet / URDF 碰撞模型做完整碰撞检测。

URDF + IK 求解使用 `soarm101_ik` 环境：

```powershell
conda activate soarm101_ik
python .\tools\ik\solve_so101_ik.py --initial shoulder_pan=0 shoulder_lift=0 elbow_flex=0 wrist_flex=0 wrist_roll=0 --dx 0.005 --dy 0 --dz 0
```

IK 脚本会输出下一步可用于 `send_joint_action.py` 的 dry-run 命令。

## 安全规则

- 上电前确认：驱动板 USB 模式跳线正确、外部电源电压正确、机械臂周围无阻挡。
- 低层点动工具默认不会运动，必须加 `--yes` 才会发送动作。
- ID5 `wrist_roll` 在当前实物上有机械限位，IK 默认冻结它，不让它参与大幅优化。
- `config/forbidden_zone.json` 是人工记录边界生成的近似禁入区，只作为回收路径安全过滤，不等价于精确碰撞模型。
- 不运行微雪官方大幅度写入示例；仓库中只保留我们实际使用的安全工具。
- 发现卡滞、撞限位、异常持续旋转时，立刻断电或执行 `tools/stservo/torque_off.py`。

## 后续主线

1. 保持六舵机通信稳定，定期用 `scan_ids.py` 和 `read_so101_observation.py` 检查。
2. 维护 LeRobot calibration 文件，尤其是 ID5 的中位和软件限位。
3. 用 `solve_so101_ik.py` 验证末端位置控制逻辑。
4. 上 autolabor2.5 / Raspberry Pi 3B 后拆成 ROS 节点：视觉识别、坐标转换、IK、机械臂驱动、安全过滤。
