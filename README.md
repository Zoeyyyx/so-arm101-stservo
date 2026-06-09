# 基于六自由度机械臂的视觉井字棋对弈系统

本项目实现了一个基于摄像头和六自由度机械臂的井字棋对弈系统。系统可以识别纸面九宫格棋盘上的人类棋子，使用井字棋策略计算机械臂下一步落子位置，再调用机械臂轨迹规划和舵机控制流程完成落子。对局结束后，机械臂会根据胜利、平局或失败执行对应的结算动作。

项目面向课程展示和本地联调，重点是把视觉识别、棋局决策、坐标映射、运动规划和执行反馈串成一个完整闭环。

## 功能特点

- 摄像头识别 A4 纸面九宫格棋盘。
- 透视变换校正棋盘区域。
- 基于 HSV 阈值识别人类棋子和机械臂棋子。
- 使用带深度权重的 minimax 策略选择机械臂落子。
- 支持胜负概率预测和提前平局判断。
- 将井字棋格子编号映射到机械臂末端坐标。
- 复用双 hit 动作完成取子和落子。
- 支持胜利、平局、失败三类结算动作。
- 提供标定、调试、dry-run 和真实执行脚本。

## 硬件与环境

当前工程按以下硬件调试：

- SO-ARM101 / SO101 结构机械臂
- 6 个 STS3215 总线舵机
- USB 串口舵机驱动板
- 普通 USB 摄像头
- A4 纸面井字棋棋盘
- 红色棋子代表人类，黑色棋子代表机械臂

默认串口配置见 `config/servo_map.json`，当前默认端口为 `COM5`，波特率为 `1000000`。

Python 最小依赖：

```powershell
pip install -r requirements.txt
```

视觉功能需要 OpenCV；机械臂 IK 和 LeRobot 相关功能可按本地环境单独配置。

## 棋盘约定

棋盘编号：

```text
0 1 2
3 4 5
6 7 8
```

棋子状态：

```text
X = 人类棋子
O = 机械臂棋子
. = 空格
? = 视觉结果不确定
```

系统流程：

```text
摄像头画面
-> 棋盘透视矫正
-> 3x3 格子颜色识别
-> 棋盘状态 X/O/.
-> 策略模块计算落子
-> 格子编号转换为机械臂坐标
-> 轨迹规划与舵机执行
-> 对局结算动作
```

## 项目结构

```text
config/
  tictactoe_vision.yaml       视觉标定和颜色阈值
  tictactoe_arm.yaml          井字棋格子到机械臂坐标的映射
  tictactoe_settlement.json   胜负结算动作配置
  hit_action.json             双 hit 落子动作参数
  servo_map.json              舵机 ID、串口和关节说明

tictactoe/
  vision.py                   棋盘透视矫正、颜色识别、稳定帧过滤
  strategy.py                 minimax 策略、概率预测、平局判断
  game_manager.py             单步对局流程管理
  arm_player.py               格子编号到机械臂动作的适配
  settlement_actions.py       胜负结算动作

scripts/
  calibrate_board_vision.py   标定棋盘四角
  debug_board_vision.py       调试视觉识别
  calibrate_arm_cells.py      标定 9 个格子的机械臂落点
  test_play_cell.py           单独测试某一格落子
  test_settlement_action.py   单独测试结算动作
  run_tictactoe_game.py       对局主程序

tools/
  stservo/                    STS3215 总线扫描、读位置、关扭矩等工具
  arm_control/                IK、轨迹规划、安全检查和执行控制
  ik/                         IK 调试脚本
  lerobot/                    LeRobot 相关辅助脚本

tests/                        核心逻辑测试
vendor/waveshare_stservo/     随项目保留的微雪 STServo SDK
```

## 视觉标定

固定摄像头、A4 纸和棋盘位置后，点击棋盘四角：

```text
左上 -> 右上 -> 右下 -> 左下
```

使用摄像头标定：

```powershell
python .\scripts\calibrate_board_vision.py --camera-index 0
```

使用图片标定：

```powershell
python .\scripts\calibrate_board_vision.py --image .\board.jpg
```

调试识别结果：

```powershell
python .\scripts\debug_board_vision.py --camera-index 0
```

## 机械臂格子标定

查看当前 9 个格子的落点配置：

```powershell
python .\scripts\calibrate_arm_cells.py --print
```

按规则网格生成格子坐标：

```powershell
python .\scripts\calibrate_arm_cells.py --generate-grid `
  --origin-x 0.240 --origin-y 0.040 `
  --col-dx 0.000 --col-dy -0.040 `
  --row-dx -0.050 --row-dy 0.000 `
  --z-press 0.010 --z-above 0.110 `
  --yes
```

生成结果写入 `config/tictactoe_arm.yaml`。真实执行前建议先单独测试中心格。

## 策略说明

机械臂下棋决策位于 `tictactoe/strategy.py`。真实落子使用 `best_move()`，核心是完整 minimax 枚举：

```text
机械臂胜利: 10 - depth
人类胜利: depth - 10
平局: 0
```

这种带深度权重的评分方式可以让机械臂优先选择更快获胜的走法；如果无法避免失败，则尽量延迟失败。

胜负概率预测是另一套演示逻辑：从当前局面继续递归枚举，假设双方后续在合法空格中均匀随机落子，统计人类胜、机械臂胜和平局概率。

## 运行方式

只输入棋盘状态，查看策略决策：

```powershell
python .\scripts\run_tictactoe_game.py --board "X........"
```

摄像头单帧识别并决策：

```powershell
python .\scripts\run_tictactoe_game.py --camera-once --camera-index 0 --show-window
```

实时视觉对局，只规划不执行：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --plan
```

实时视觉对局并执行机械臂动作：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --execute
```

禁用结算动作：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --execute --no-settlement
```

## 单独测试

规划中心格落子，不真实运动：

```powershell
python .\scripts\test_play_cell.py --cell 4 --port COM5 --plan
```

真实执行中心格落子：

```powershell
python .\scripts\test_play_cell.py --cell 4 --port COM5 --execute
```

测试结算动作：

```powershell
python .\scripts\test_settlement_action.py --action victory --port COM5 --execute
python .\scripts\test_settlement_action.py --action draw_handshake --port COM5 --execute
python .\scripts\test_settlement_action.py --action defeat_nod --port COM5 --execute
```

动作含义：

```text
victory        机械臂胜利，执行庆祝动作
draw_handshake 平局，执行握手动作
defeat_nod     机械臂失败，执行低头动作
```

## 测试

运行核心单元测试：

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
$env:PYTHONPATH=(Get-Location).Path
pytest -q tests -p no:cacheprovider
```

测试覆盖策略判断、胜负概率、视觉配置、对局流程和格子坐标适配。

## 安全提示

- 真实执行前先使用 `--plan` 检查轨迹。
- 摄像头、棋盘或机械臂位置变化后，需要重新检查视觉标定和格子坐标。
- 对局中出现 `?` 时系统会跳过当前帧，避免误落子。
- 机械臂运动范围内不要放置手或杂物。
- 出现卡滞、碰撞风险或异常持续旋转时，优先断电；能通信时可运行 `tools/stservo/torque_off.py` 关闭扭矩。
