"""Calibrate the four tic-tac-toe board corners.

Click order: top-left, top-right, bottom-right, bottom-left.

Examples:

    python scripts/calibrate_board_vision.py --camera-index 0
    python scripts/calibrate_board_vision.py --image board.jpg
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.vision import load_vision_config, require_cv2, save_vision_config  # noqa: E402


POINT_NAMES = ("top-left", "top-right", "bottom-right", "bottom-left")
WINDOW_NAME = "calibrate tictactoe board"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Click four board corners and write the vision config.")
    parser.add_argument("--config", default="config/tictactoe_vision.yaml", help="Vision config path.")
    parser.add_argument("--output", help="Output config path. Defaults to --config.")
    parser.add_argument("--image", help="Use an image instead of opening the camera.")
    parser.add_argument("--camera-index", type=int, help="Override camera_index from the config.")
    return parser.parse_args()


def window_is_closed(cv2, window_name: str) -> bool:
    """Return True if the OpenCV window has been closed."""

    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def load_frame(args: argparse.Namespace):
    cv2, _ = require_cv2()
    config = load_vision_config(args.config)
    if args.image:
        frame = cv2.imread(str(Path(args.image)))
        if frame is None:
            raise SystemExit(f"Could not read image: {args.image}")
        return frame, config

    if args.camera_index is not None:
        config.camera_index = args.camera_index
    cap = cv2.VideoCapture(config.camera_index)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera: {config.camera_index}")
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("Camera read failed.")
    return frame, config


def main() -> None:
    args = parse_args()
    cv2, _ = require_cv2()
    frame, config = load_frame(args)
    points: list[tuple[int, int]] = []

    def redraw() -> None:
        preview = frame.copy()
        for index, point in enumerate(points):
            cv2.circle(preview, point, 6, (0, 255, 255), -1)
            cv2.putText(
                preview,
                f"{index + 1}:{POINT_NAMES[index]}",
                (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        if len(points) > 1:
            for a, b in zip(points, points[1:]):
                cv2.line(preview, a, b, (0, 255, 255), 2)
        if len(points) == 4:
            cv2.line(preview, points[3], points[0], (0, 255, 255), 2)
        cv2.imshow(WINDOW_NAME, preview)

    def on_mouse(event, x, y, _flags, _userdata) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or len(points) >= 4:
            return
        points.append((x, y))
        print(f"Recorded {POINT_NAMES[len(points) - 1]}: ({x}, {y})")
        redraw()

    print("Click corners in order: top-left, top-right, bottom-right, bottom-left.")
    print("Press r to reset, s to save, q or close window to exit.")
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    redraw()

    try:
        while True:
            if window_is_closed(cv2, WINDOW_NAME):
                break

            key = cv2.waitKey(20) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):
                points.clear()
                print("Reset. Click the four corners again.")
                redraw()
            if key == ord("s"):
                if len(points) != 4:
                    print("Need exactly 4 corner points before saving.")
                    continue
                config.board_corners = [(float(x), float(y)) for x, y in points]
                output = args.output or args.config
                save_vision_config(config, output)
                print(f"Saved board corners to: {output}")
                break
    except KeyboardInterrupt:
        print("Ctrl+C received. Exiting board calibration.")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
