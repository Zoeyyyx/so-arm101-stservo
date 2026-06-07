# 基于 SO-ARM101 / SO101 机械臂的视觉井字棋对弈系统

本项目是一个机械臂视觉井字棋下棋系统。系统通过摄像头识别 A4 纸面上的九宫格棋盘，实时判断红色人类棋子和黑色机械臂棋子的位置，使用井字棋策略模块决定机械臂下一步落子，并调用现有机械臂动作控制流程完成落子动作。对局结束后，机械臂会根据胜负结果执行对应的结算动作。

当前系统面向演示和联调：重点是视觉识别稳定、策略判断明确、动作流程可复现、结算动作可单独测试。

## 系统流程

```text
摄像头画面
→ 棋盘四角透视矫正
→ 3x3 格子颜色识别
→ 当前棋盘状态 X/O/.
→ 胜负概率预测与提前平局判断
→ 策略模块选择机械臂落子格
→ 格子编号转换为机械臂坐标
→ 双 hit 落子动作
→ 胜利 / 平局 / 失败结算动作
```

棋子约定：

```text
X = 人类红色棋子
O = 机械臂黑色棋子
. = 空格
? = 视觉不确定
```

棋盘编号：

```text
0 1 2
3 4 5
6 7 8
```

## 目录结构

```text
config/
  tictactoe_vision.yaml       视觉识别配置，包括棋盘四角、颜色阈值、稳定帧数
  tictactoe_arm.yaml          井字棋格子到机械臂落点坐标的映射
  tictactoe_settlement.json   胜负结算动作配置
  hit_action.json             双 hit 落子动作参数
  home_pose.json              机械臂收回姿态
  ready_pose.json             机械臂展开准备姿态

tictactoe/
  vision.py                   棋盘透视矫正、颜色识别、稳定帧过滤
  strategy.py                 井字棋策略、胜负概率预测、提前平局判断
  game_manager.py             单步对局状态管理
  arm_player.py               格子编号到机械臂落子命令的适配
  settlement_actions.py       对局结算动作

scripts/
  calibrate_board_vision.py   点击棋盘四角，生成视觉标定
  debug_board_vision.py       调试棋盘和棋子识别
  calibrate_arm_cells.py      标定 9 个格子的机械臂坐标
  test_play_cell.py           单独测试机械臂落某一格
  test_settlement_action.py   单独测试胜负结算动作
  run_tictactoe_game.py       实时井字棋对局主程序

tools/arm_control/
  hit_target_action.py        机械臂落子动作 CLI 入口
  core/                       轨迹规划、IK、安全检查、舵机执行封装
```

## 环境准备

在 Windows PowerShell 中：

```powershell
conda activate lerobot
cd C:\A_projects\260302_ROS_Logistics_Trolley\MechanicalArm\STServo_Python
$env:PYTHONIOENCODING='utf-8'
```

默认机械臂串口：

```text
COM5
```

## 视觉标定

固定 A4 横向纸面和摄像头位置后，点击棋盘四角。点击顺序：

```text
top-left → top-right → bottom-right → bottom-left
```

相机标定：

```powershell
python .\scripts\calibrate_board_vision.py --camera-index 0
```

使用图片标定：

```powershell
python .\scripts\calibrate_board_vision.py --image .\board.jpg
```

标定结果写入：

```text
config/tictactoe_vision.yaml
```

视觉调试：

```powershell
python .\scripts\debug_board_vision.py --camera-index 0
```

单帧调试并保存图像：

```powershell
python .\scripts\debug_board_vision.py --camera-index 0 --once --save-debug .\debug_board.jpg
```

## 机械臂格子标定

当前 9 个格子的坐标配置在：

```text
config/tictactoe_arm.yaml
```

打印当前格子落点：

```powershell
python .\scripts\calibrate_arm_cells.py --print
```

按规则网格生成 9 格坐标：

```powershell
python .\scripts\calibrate_arm_cells.py --generate-grid `
  --origin-x 0.240 --origin-y 0.040 `
  --col-dx 0.000 --col-dy -0.040 `
  --row-dx -0.050 --row-dy 0.000 `
  --z-press 0.010 --z-above 0.110
```

确认写入配置时追加：

```powershell
--yes
```

## 单格落子测试

先 dry-run 规划，不真实运动：

```powershell
python .\scripts\test_play_cell.py --cell 4 --port COM5 --plan
```

真实执行某一格落子：

```powershell
python .\scripts\test_play_cell.py --cell 4 --port COM5 --execute
```

井字棋层不会重写机械臂动作逻辑。`test_play_cell.py` 会把格子编号转换成 `hit_target_action.py` 的目标坐标，并复用现有双 hit 落子流程。

## 对局策略与预测

策略模块位于：

```text
tictactoe/strategy.py
```

机械臂真实落子仍使用 `best_move()`，优先级是：

```text
能赢则赢
必须防守则堵
否则按 minimax 选择稳定落点
```

胜负概率预测使用另一套演示口径：

```text
从当前局面开始，轮到谁，谁就在所有合法空格中均匀随机落子。
递归枚举所有可能对局，统计 X 胜 / O 胜 / 平局概率。
```

提前平局不再单纯依赖“平局概率 100%”，而是使用结构判断：

```text
棋盘未满
无人已经获胜
X 在剩余空格和剩余落子次数内无法形成任意三连
O 在剩余空格和剩余落子次数内也无法形成任意三连
```

满足时状态为：

```text
forced_draw
```

系统会提前结束对局并执行平局结算动作。

## 对局运行

手动输入棋盘，只看决策：

```powershell
python .\scripts\run_tictactoe_game.py --board "X........"
```

视觉单次识别并决策：

```powershell
python .\scripts\run_tictactoe_game.py --camera-once --camera-index 0 --show-window
```

实时视觉对局 dry-run，连接机械臂并做规划但不执行动作：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --plan
```

实时视觉对局真实执行：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --execute
```

禁用结算动作：

```powershell
python .\scripts\run_tictactoe_game.py --camera-live --camera-index 0 --show-window --port COM5 --execute --no-settlement
```

实时模式中，程序只在棋盘连续稳定若干帧后处理一次局面。视觉结果中出现 `?` 时会跳过，避免机械臂误落子。

## 机械臂落子动作

当前落子采用双 hit 流程：

```text
home
→ ready
→ hit1_above
→ hit1_down
→ hit1_hold
→ hit1_up
→ hit2_above
→ hit2_down
→ hit2_hold
→ hit2_up
→ return_ready
→ return_home
```

其中：

```text
hit1 = 取子/加载点动作
hit2 = 目标棋盘格动作
```

`hit2` 的 x/y/z 来自 `config/tictactoe_arm.yaml` 中对应 cell 的配置。动作速度、停顿、等待策略、ID4 单关节速度覆盖等在 `config/hit_action.json` 中配置。

## 对局结算动作

结算动作配置：

```text
config/tictactoe_settlement.json
```

动作映射：

```text
机械臂胜利 robot_win → victory
平局 draw / forced_draw → draw_handshake
人类胜利 human_win → defeat_nod
```

单独预览结算动作：

```powershell
python .\scripts\test_settlement_action.py --action victory
python .\scripts\test_settlement_action.py --action draw_handshake
python .\scripts\test_settlement_action.py --action defeat_nod
```

真实执行结算动作：

```powershell
python .\scripts\test_settlement_action.py --action victory --port COM5 --execute
python .\scripts\test_settlement_action.py --action draw_handshake --port COM5 --execute
python .\scripts\test_settlement_action.py --action defeat_nod --port COM5 --execute
```

当前动作含义：

```text
victory
  回 home
  gripper 在 2~35 度开合
  wrist_roll 以 home 姿态为中心往复摆动
  动作本体持续 5 秒
  返回 home

draw_handshake
  回 home
  wrist_flex 快速到 home + 90 度中心位
  围绕中心做握手摆动
  摆动本体持续 5 秒
  返回 home

defeat_nod
  快速回到 home
  wrist_flex 快速到 home + 135 度
  保持该位置 5 秒
  快速返回 home
```

## 常用联调顺序

推荐按下面顺序推进：

```text
1. calibrate_board_vision.py
2. debug_board_vision.py
3. calibrate_arm_cells.py --print
4. test_play_cell.py --cell 4 --plan
5. test_play_cell.py --cell 4 --execute
6. test_settlement_action.py --action victory --execute
7. run_tictactoe_game.py --camera-live --plan
8. run_tictactoe_game.py --camera-live --execute
```

## 测试

运行核心测试：

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
$env:PYTHONPATH=(Get-Location).Path
pytest -q tests -p no:cacheprovider
```

当前测试覆盖：

```text
井字棋策略与胜负预测
对局状态管理
视觉识别配置与棋盘格式
格子坐标到机械臂命令的适配
```

## 安全提示

- 真实执行前先用 `--plan` 或单独测试脚本确认轨迹。
- 相机、纸面、机械臂位置变化后需要重新检查视觉标定和格子坐标。
- 结算动作会直接控制 `wrist_roll`、`wrist_flex`、`gripper` 等关节，首次修改参数后建议先单独运行 `test_settlement_action.py`。
- 对局中出现识别不确定 `?` 时，系统会跳过当前帧，不应强行执行机械臂动作。
- 真实对局时保持机械臂运动范围内无人手和杂物。
