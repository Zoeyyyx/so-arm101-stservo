"""运行井字棋机械臂 demo。

手动测试：

    python scripts/run_tictactoe_game.py --board "X........"

实时视觉对局：

    python scripts/run_tictactoe_game.py --camera-live

默认只输出决策和等价机械臂命令，不连接舵机。
加 --plan 才连接机械臂做 dry-run 规划。
加 --execute 才真实执行动作。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.arm_player import ArmCellPlayer, load_arm_config  # noqa: E402
from tictactoe.game_manager import TicTacToeGameManager, result_summary_lines  # noqa: E402
from tictactoe.strategy import check_winner, is_draw  # noqa: E402
from tictactoe.vision import (  # noqa: E402
    UNKNOWN,
    BoardStabilityFilter,
    draw_debug_overlay,
    format_vision_board,
    load_vision_config,
    recognize_frame,
    require_cv2,
    warp_board,
)


DEFAULT_ARM_CONFIG = PROJECT_ROOT / "config" / "tictactoe_arm.yaml"
DEFAULT_VISION_CONFIG = PROJECT_ROOT / "config" / "tictactoe_vision.yaml"
WINDOW_NAME = "tictactoe live game"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="运行井字棋机械臂 demo")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--board", help='手动输入 9 格棋盘，例如 "X........"')
    source.add_argument("--image", help="从图片识别棋盘")
    source.add_argument("--camera-once", action="store_true", help="从摄像头读取稳定棋盘后决策一次")
    source.add_argument("--camera-live", action="store_true", help="实时连续检测棋盘并自动给出机械臂下一步")

    parser.add_argument("--arm-config", default=str(DEFAULT_ARM_CONFIG), help="井字棋机械臂落点配置")
    parser.add_argument("--vision-config", default=str(DEFAULT_VISION_CONFIG), help="井字棋视觉配置")
    parser.add_argument("--camera-index", type=int, help="覆盖视觉配置里的摄像头编号")
    parser.add_argument("--max-frames", type=int, default=80, help="camera-once 最多读取多少帧")
    parser.add_argument("--live-stable-frames", type=int, help="实时模式连续多少帧一致后确认棋盘")
    parser.add_argument("--decision-cooldown", type=float, default=1.0, help="两次决策之间的最短间隔，单位秒")
    parser.add_argument("--show-window", action="store_true", help="实时模式显示调试窗口")
    parser.add_argument("--port", help="覆盖机械臂 COM 口，例如 COM5")
    parser.add_argument("--plan", action="store_true", help="连接机械臂并做 dry-run 规划")
    parser.add_argument("--execute", action="store_true", help="真实执行机械臂落子动作")
    return parser


def window_is_closed(cv2, window_name: str) -> bool:
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def print_result(result) -> None:
    for line in result_summary_lines(result):
        if line.startswith(str(sys.executable)):
            print(subprocess.list2cmdline(result.command))
        else:
            print(line)


def read_board_from_image(image_path: str, vision_config_path: str):
    cv2, _ = require_cv2()
    config = load_vision_config(vision_config_path)
    image = cv2.imread(str(Path(image_path)))
    if image is None:
        raise SystemExit(f"无法读取图片: {image_path}")
    result = recognize_frame(image, config)
    print("图片识别棋盘:")
    print(format_vision_board(result.board))
    return result.board


def read_board_from_camera_once(args: argparse.Namespace):
    cv2, _ = require_cv2()
    config = load_vision_config(args.vision_config)
    if args.camera_index is not None:
        config.camera_index = args.camera_index

    cap = cv2.VideoCapture(config.camera_index)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头: {config.camera_index}")

    stable = BoardStabilityFilter(config.stable_frame_count)
    last_board = None
    try:
        for _ in range(max(1, args.max_frames)):
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("摄像头读取失败")
            result = recognize_frame(frame, config)
            last_board = result.board
            stable_board = stable.update(result.board)
            if stable_board is not None and UNKNOWN not in stable_board:
                print("摄像头稳定棋盘:")
                print(format_vision_board(stable_board))
                return stable_board
    finally:
        cap.release()

    if last_board is None:
        raise SystemExit("没有读到有效棋盘帧")
    print("没有达到稳定帧数量，使用最后一帧；建议检查光照或阈值。")
    print(format_vision_board(last_board))
    return last_board


def board_changed_enough(previous: tuple[str, ...] | None, current: tuple[str, ...]) -> bool:
    """实时模式的去重过滤。"""

    if previous is None:
        return True
    return tuple(previous) != tuple(current)


def is_game_over(board: tuple[str, ...]) -> bool:
    return check_winner(board) is not None or is_draw(board)


def run_live_camera(args: argparse.Namespace, manager: TicTacToeGameManager) -> None:
    cv2, _ = require_cv2()
    config = load_vision_config(args.vision_config)
    if args.camera_index is not None:
        config.camera_index = args.camera_index
    if args.live_stable_frames is not None:
        config.stable_frame_count = max(1, int(args.live_stable_frames))

    cap = cv2.VideoCapture(config.camera_index)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头: {config.camera_index}")

    stable = BoardStabilityFilter(config.stable_frame_count)
    last_confirmed_board: tuple[str, ...] | None = None
    last_decision_time = 0.0
    print("实时对局已启动。")
    print("请你放红棋；程序只在棋盘稳定后输出黑棋落点。按 q / Ctrl+C / 关闭窗口退出。")
    print(f"稳定帧数: {config.stable_frame_count}，决策冷却: {args.decision_cooldown:.2f}s")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("摄像头读取失败")

            vision_result = recognize_frame(frame, config)
            stable_board = stable.update(vision_result.board)

            if args.show_window:
                warped = warp_board(frame, config)
                debug = draw_debug_overlay(warped, vision_result, config)
                cv2.imshow(WINDOW_NAME, debug)
                if window_is_closed(cv2, WINDOW_NAME):
                    break
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            else:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            if stable_board is None:
                continue
            stable_board = tuple(stable_board)

            # 手、阴影、棋子移动过程中容易出现 ?，直接跳过，不触发对局逻辑。
            if UNKNOWN in stable_board:
                continue
            if not board_changed_enough(last_confirmed_board, stable_board):
                continue
            if time.monotonic() - last_decision_time < float(args.decision_cooldown):
                continue

            result = manager.process_board(
                stable_board,
                plan=args.plan or args.execute,
                execute=args.execute,
                port=args.port,
            )
            last_confirmed_board = stable_board
            last_decision_time = time.monotonic()

            print("\n检测到新的稳定棋盘:")
            print(format_vision_board(stable_board))
            print_result(result)

            if result.status in {"robot_move_ready", "robot_move_needs_arm_calibration"} and result.robot_cell is not None:
                print(f"请把黑棋放到 cell {result.robot_cell}。")
                print("放好后继续下红棋，程序会等待下一次稳定变化。")

            if result.success and is_game_over(stable_board):
                print("对局已结束。")
                break
    except KeyboardInterrupt:
        print("收到 Ctrl+C，退出实时对局。")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def read_board(args: argparse.Namespace):
    if args.board:
        return args.board
    if args.image:
        return read_board_from_image(args.image, args.vision_config)
    return read_board_from_camera_once(args)


def main() -> None:
    args = build_parser().parse_args()
    arm_config = load_arm_config(args.arm_config)
    if args.port:
        arm_config.port = args.port

    player = ArmCellPlayer(arm_config, repo_root=PROJECT_ROOT)
    manager = TicTacToeGameManager(player)

    if args.camera_live:
        run_live_camera(args, manager)
        return

    board = read_board(args)
    result = manager.process_board(board, plan=args.plan or args.execute, execute=args.execute, port=args.port)
    print_result(result)

    if not result.success:
        raise SystemExit(1)
    if result.status in {"robot_move_ready", "robot_move_needs_arm_calibration"} and not args.plan and not args.execute:
        print("当前只是命令预览；需要 dry-run 规划时加 --plan，真实执行时加 --execute。")


if __name__ == "__main__":
    main()
