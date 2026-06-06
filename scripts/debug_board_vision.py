"""调试井字棋棋盘视觉识别。

常用方式：

    python scripts/debug_board_vision.py --image board.jpg
    python scripts/debug_board_vision.py --camera-index 0

先运行 calibrate_board_vision.py 标定四角，再运行本脚本看 9 格识别结果。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.vision import (  # noqa: E402
    BoardStabilityFilter,
    analyze_warped_board,
    draw_debug_overlay,
    format_vision_board,
    load_vision_config,
    recognize_frame,
    require_cv2,
    warp_board,
)


WINDOW_NAME = "tictactoe vision debug"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="调试井字棋棋盘视觉识别")
    parser.add_argument("--config", default="config/tictactoe_vision.yaml", help="视觉配置文件路径")
    parser.add_argument("--image", help="使用图片调试，而不是打开摄像头")
    parser.add_argument("--camera-index", type=int, help="覆盖配置里的 camera_index")
    parser.add_argument("--save-debug", help="保存带九宫格识别结果的调试图片")
    parser.add_argument("--once", action="store_true", help="摄像头模式只识别一帧后退出")
    return parser.parse_args()


def window_is_closed(cv2, window_name: str) -> bool:
    """判断 OpenCV 窗口是否已经被用户关闭。"""

    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def print_result(board, stable_board=None, cells=None) -> None:
    print("当前识别棋盘:")
    print(format_vision_board(board))
    if stable_board is not None:
        print("稳定棋盘:")
        print(format_vision_board(stable_board))
    if cells is not None:
        print("每格像素统计: H=红色像素, R=黑色像素")
        for cell in cells:
            print(
                f"  cell {cell.index}: state={cell.state} "
                f"H={cell.human_pixels} R={cell.robot_pixels}"
            )


def run_image_mode(args: argparse.Namespace) -> None:
    cv2, _ = require_cv2()
    config = load_vision_config(args.config)
    image = cv2.imread(str(Path(args.image)))
    if image is None:
        raise SystemExit(f"无法读取图片: {args.image}")

    warped = warp_board(image, config)
    result = analyze_warped_board(warped, config)
    print_result(result.board, cells=result.cells)

    debug = draw_debug_overlay(warped, result, config)
    if args.save_debug:
        cv2.imwrite(args.save_debug, debug)
        print(f"已保存调试图片: {args.save_debug}")
        return

    cv2.imshow(WINDOW_NAME, debug)
    print("按任意键或关闭窗口退出。")
    while not window_is_closed(cv2, WINDOW_NAME):
        if cv2.waitKey(20) != -1:
            break
    cv2.destroyAllWindows()


def run_camera_mode(args: argparse.Namespace) -> None:
    cv2, _ = require_cv2()
    config = load_vision_config(args.config)
    if args.camera_index is not None:
        config.camera_index = args.camera_index

    cap = cv2.VideoCapture(config.camera_index)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头: {config.camera_index}")

    stable = BoardStabilityFilter(config.stable_frame_count)
    print("按 q 或关闭窗口退出；按空格打印当前棋盘。")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("摄像头读取失败")

            result = recognize_frame(frame, config)
            stable_board = stable.update(result.board)
            warped = warp_board(frame, config)
            debug = draw_debug_overlay(warped, result, config)
            cv2.imshow(WINDOW_NAME, debug)

            if window_is_closed(cv2, WINDOW_NAME):
                break

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") or args.once:
                print_result(result.board, stable_board, result.cells)
                if args.once:
                    break
    except KeyboardInterrupt:
        print("收到 Ctrl+C，退出视觉调试。")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    args = parse_args()
    if args.image:
        run_image_mode(args)
    else:
        run_camera_mode(args)


if __name__ == "__main__":
    main()
