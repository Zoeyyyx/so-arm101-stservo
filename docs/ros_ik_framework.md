# SO101 视觉打靶项目：URDF + IK + ROS 节点框架

当前仓库先在 Windows 上跑通 URDF + IK + LeRobot 控制。后续迁移到 autolabor2.5 / Raspberry Pi 3B 时，再把这些功能拆成 ROS 节点。

## 当前已建立的链路

1. 低层通信检查
   - 工具：`tools/stservo/scan_ids.py`
   - 目的：确认 `COM5` 上 1-6 号 STS3215 都能回包。

2. LeRobot 机械臂驱动
   - 工具：`tools/lerobot/read_so101_observation.py`
   - 工具：`tools/lerobot/send_joint_action.py`
   - 目的：使用 LeRobot calibration 读取关节角，并调用 `robot.send_action()` 发送目标。

3. URDF + IK 求解
   - URDF：`assets/urdf/so101_new_calib.urdf`
   - 工具：`tools/ik/solve_so101_ik.py`
   - 目的：输入末端位置偏移，输出可交给 LeRobot 的关节目标。

## ROS 节点建议

后续不要把 YOLO、IK、串口控制写在一个大节点里，建议拆成：

1. `yolo_target_node`
   - 订阅：相机图像
   - 发布：靶子像素坐标、颜色类别、置信度

2. `target_projection_node`
   - 订阅：靶子像素坐标
   - 输入：相机内参、相机到机械臂基座的外参、地面平面参数
   - 发布：机械臂基座坐标系下的目标点

3. `so101_ik_node`
   - 订阅：目标点
   - 使用：SO101 URDF + IK
   - 发布：关节目标

4. `so101_arm_driver_node`
   - 订阅：关节目标
   - 调用：LeRobot `robot.send_action()`
   - 发布：当前关节状态

5. `safety_filter_node`
   - 可独立，也可先内置在 arm driver 中
   - 负责：关节限位、单步最大变化、ID5 `wrist_roll` 限幅、急停

## 当前 Windows 测试流程

先读取当前机械臂关节角：

```powershell
conda activate lerobot
python .\tools\lerobot\read_so101_observation.py --port COM5 --id soarm101_follower --count 1
```

把读到的当前角度填进 IK 的 `--initial`。下面数字只是示例：

```powershell
conda activate soarm101_ik
python .\tools\ik\solve_so101_ik.py --initial shoulder_pan=0 shoulder_lift=0 elbow_flex=0 wrist_flex=0 wrist_roll=0 --dx 0.005 --dy 0 --dz 0
```

IK 脚本会输出一条 `send_joint_action.py` 命令。先在 `lerobot` 环境里 dry-run：

```powershell
conda activate lerobot
python .\tools\lerobot\send_joint_action.py --port COM5 --id soarm101_follower --max-relative 5 --set shoulder_pan=0 shoulder_lift=2 elbow_flex=-1 wrist_flex=-2 wrist_roll=0 gripper=0
```

确认目标角度和机械空间安全后，再加 `--yes`。

## 后续接视觉时要补的关键问题

- 相机标定：得到相机内参。
- 外参标定：得到相机坐标系到机械臂基座坐标系的变换。
- 地面平面建模：YOLO 只能给像素坐标，必须投影到实际地面坐标。
- 安全策略：目标点不可达、IK 解跳变、关节接近限位时必须拒绝动作。
- 打靶动作：末端到达目标附近后，需要单独设计击打/触碰动作，不要让视觉节点直接控制舵机原始位置。

## 当前限制

- Windows 上 LeRobot 的 `placo` 后端存在 DLL 导入问题，当前使用 `ikpy` 先跑通 URDF + IK。
- ID5 `wrist_roll` 有物理限位，当前 IK 默认冻结它。
- Raspberry Pi 3B 算力有限，YOLOv8 后续可能需要轻量模型或外部上位机协同。

