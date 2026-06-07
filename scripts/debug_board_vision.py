"""Debug tic-tac-toe board vision recognition.

Common usage:

    python scripts/debug_board_vision.py --image board.jpg
    python scripts/debug_board_vision.py --camera-index 0

Run calibrate_board_vision.py first, then use this script to inspect 3x3 recognition.
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
    parser = argparse.ArgumentParser(description="Debug tic-tac-toe board vision recognition.")
    parser.add_argument("--config", default="config/tictactoe_vision.yaml", help="Vision config path.")
    parser.add_argument("--image", help="Use an image instead of opening the camera.")
    parser.add_argument("--camera-index", type=int, help="Override camera_index from the config.")
    parser.add_argument("--save-debug", help="Save the debug image with 3x3 recognition overlay.")
    parser.add_argument("--once", action="store_true", help="Camera mode: process one frame and exit.")
    return parser.parse_args()


def window_is_closed(cv2, window_name: str) -> bool:
    """Return True if the OpenCV window has been closed."""

    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def print_result(board, stable_board=None, cells=None) -> None:
    print("Current board:")
    print(format_vision_board(board))
    if stable_board is not None:
        print("Stable board:")
        print(format_vision_board(stable_board))
    if cells is not None:
        print("Per-cell pixel stats: H=red pixels, R=black pixels")
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
        raise SystemExit(f"Could not read image: {args.image}")

    warped = warp_board(image, config)
    result = analyze_warped_board(warped, config)
    print_result(result.board, cells=result.cells)

    debug = draw_debug_overlay(warped, result, config)
    if args.save_debug:
        cv2.imwrite(args.save_debug, debug)
        print(f"Saved debug image: {args.save_debug}")
        return

    cv2.imshow(WINDOW_NAME, debug)
    print("Press any key or close the window to exit.")
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
        raise SystemExit(f"Could not open camera: {config.camera_index}")

    stable = BoardStabilityFilter(config.stable_frame_count)
    print("Press q or close the window to exit; press Space to print the current board.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("Camera read failed.")

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
        print("Ctrl+C received. Exiting vision debug.")
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
