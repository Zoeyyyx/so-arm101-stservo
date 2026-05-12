# LeRobot 标定与限位说明

本文件记录 SO101 follower 的 LeRobot 标定原则，重点说明 calibration 文件位置、六舵机映射和 ID5 `wrist_roll` 的限位处理。

## 标定文件位置

LeRobot 默认把本机标定文件写到用户目录：

```powershell
C:\Users\zyx\.cache\huggingface\lerobot\calibration\robots\so_follower\soarm101_follower.json
```

查看当前标定文件：

```powershell
Get-Content C:\Users\zyx\.cache\huggingface\lerobot\calibration\robots\so_follower\soarm101_follower.json
```

## 正确的六舵机映射

| 关节名 | 舵机 ID | 说明 |
| --- | --- | --- |
| `shoulder_pan` | 1 | 底座旋转 |
| `shoulder_lift` | 2 | 肩关节抬升 |
| `elbow_flex` | 3 | 肘关节弯曲 |
| `wrist_flex` | 4 | 腕部俯仰 |
| `wrist_roll` | 5 | 腕部旋转 |
| `gripper` | 6 | 夹爪 |

如果标定表少了 `wrist_roll`，优先检查是否误用了 `so100_follower` 或旧的 `robot.id`。

## 重新标定命令

```powershell
conda activate lerobot
lerobot-calibrate --robot.type=so101_follower --robot.port=COM5 --robot.id=soarm101_follower
```

如果怀疑旧缓存影响，可以换一个新的 `robot.id` 重新标定：

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM5 --robot.id=soarm101_6axis_test
```

## ID5 wrist_roll 特别说明

当前实物上的 ID5 有物理限位，不应强求它像无限旋转关节一样跑完整范围。项目策略是：

- 标定文件里保留 `wrist_roll`，保证 6 轴模型完整；
- 软件上收窄 ID5 的允许范围；
- IK 默认冻结 `wrist_roll`，避免视觉打靶时误让 ID5 大幅旋转；
- 需要调整 ID5 中位时使用专用脚本，不手改底层协议。

预览 ID5 中位和限位写入：

```powershell
conda activate lerobot
python .\tools\lerobot\set_wrist_roll_center.py --port COM5 --id soarm101_follower --min-deg -45 --max-deg 15
```

确认机械位置安全后再加 `--yes`：

```powershell
python .\tools\lerobot\set_wrist_roll_center.py --port COM5 --id soarm101_follower --min-deg -45 --max-deg 15 --yes
```

脚本会在写入前备份原 calibration 文件。

## 标定操作注意

- 标定时要充分移动每个可动关节，否则 `range_min` 和 `range_max` 会非常窄。
- 不要让关节硬撞机械限位，接近极限即可。
- 标定完成后先运行 observation 读取，再做小幅动作验证。
- 如果某个关节方向异常，先停止动作，检查 ID、装配方向和 calibration。

