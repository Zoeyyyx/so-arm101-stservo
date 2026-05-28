# 项目结构说明

本仓库按“硬件通信、LeRobot 控制、IK 求解、后续 ROS 集成”来划分目录。这样做的目的，是避免把临时测试脚本、官方示例和真正要复用的项目代码混在一起。

```text
.
├─ assets/
│  └─ urdf/                    SO101 机械臂 URDF 模型
├─ config/
│  └─ servo_map.json           舵机 ID 与 LeRobot 关节名映射
├─ docs/
│  ├─ hardware-status.md       当前硬件、通信和标定状态
│  ├─ calibration-notes.md     标定文件、ID5 限位、中位设置说明
│  ├─ project-structure.md     本文件
│  └─ ros_ik_framework.md      后续 ROS + IK + 视觉节点框架
├─ tools/
│  ├─ stservo/                 微雪 STServo 低层工具
│  ├─ lerobot/                 LeRobot 读取状态和发送动作工具
│  ├─ ik/                      URDF + IK 离线求解工具
│  └─ arm_control/             末端绝对坐标控制模板
├─ vendor/
│  └─ waveshare_stservo/
│     └─ STservo_sdk/          微雪官方 ST 系列舵机通信 SDK
├─ requirements.txt            低层 STServo 工具最小依赖
└─ README.md
```

## tools/stservo

这一层只处理“舵机总线是否通、ID 是否在线、原始位置是否能读、必要时关闭扭矩”。它不依赖 LeRobot，也不做复杂控制。

保留工具：

- `scan_ids.py`：扫描 1-6 号舵机是否在线。
- `read_positions.py`：读取原始位置和速度。
- `servo_status.py`：读取单个舵机模式、扭矩、电压、温度、位置。
- `torque_off.py`：关闭一个或多个舵机扭矩。
- `jog_servo.py`：低层单舵机小幅点动，仅用于排障。
- `stservo_common.py`：以上脚本共用的 SDK 加载和串口打开逻辑。

## tools/lerobot

这一层使用 LeRobot 的 SO101 follower 封装，所有角度和夹爪值都经过 LeRobot calibration。

保留工具：

- `read_so101_observation.py`：读取 LeRobot 关节状态。
- `send_joint_action.py`：发送一个或多个关节目标，默认 dry-run。
- `set_wrist_roll_center.py`：重新设置 ID5 / `wrist_roll` 中位和软件限位。

## tools/ik

这一层先在 Windows 上验证 URDF + IK，不直接控制硬件。

保留工具：

- `solve_so101_ik.py`：输入当前关节角和末端偏移，输出 LeRobot 关节目标。

## tools/arm_control

这一层是后续 ROS 机械臂驱动节点的模板雏形，负责把绝对目标位姿转换成舵机指令。

保留工具：

- `send_absolute_pose_template.py`：输入 `x/y/z/roll/pitch/yaw`，先用标定关节范围的 95% 生成安全关节包络和末端工作空间，再做 IK、关节极限检查、平滑插值，并通过微雪总线发送到 STS3215。默认 dry-run。
- `hit_target_action.py`：面向地面靶位的动作程序，保持 X/Y 为靶心，沿 Z 方向执行 hover、快速接近、减速击打、停留和离开轨迹。默认 dry-run。

## vendor

`vendor/waveshare_stservo/STservo_sdk` 是微雪官方 ST 系列舵机通信库。项目代码只调用它，不修改底层通信协议。

旧的官方示例脚本已经删除，因为它们容易绕过我们的安全开关，也会让团队误以为应该直接运行大幅度写入 demo。
