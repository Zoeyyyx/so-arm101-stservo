"""标定井字棋棋盘四角。

点击顺序固定为：左上、右上、右下、左下。

示例：

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


POINT_NAMES = ("左上", "右上", "右下", "左下")
WINDOW_NAME = "calibrate tictactoe board"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="手动点击棋盘四角并写入视觉配置")
    parser.add_argument("--config", default="config/tictactoe_vision.yaml", help="视觉配置文件路径")
    parser.add_argument("--output", help="输出配置路径，默认覆盖 --config")
    parser.add_argument("--image", help="使用图片标定，而不是打开摄像头")
    parser.add_argument("--camera-index", type=int, help="覆盖配置里的 camera_index")
    return parser.parse_args()


def window_is_closed(cv2, window_name: str) -> bool:
    """判断 OpenCV 窗口是否已经被用户关闭。"""

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
            raise SystemExit(f"无法读取图片: {args.image}")
        return frame, config

    if args.camera_index is not None:
        config.camera_index = args.camera_index
    cap = cv2.VideoCapture(config.camera_index)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头: {config.camera_index}")
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("摄像头读取失败")
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
        print(f"已记录 {POINT_NAMES[len(points) - 1]}: ({x}, {y})")
        redraw()

    print("请按顺序点击：左上、右上、右下、左下。")
    print("按 r 重置，按 s 保存，按 q 或关闭窗口退出。")
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
                print("已重置，请重新点击四角。")
                redraw()
            if key == ord("s"):
                if len(points) != 4:
                    print("还没有记录满 4 个角点，不能保存。")
                    continue
                config.board_corners = [(float(x), float(y)) for x, y in points]
                output = args.output or args.config
                save_vision_config(config, output)
                print(f"已保存棋盘四角到: {output}")
                break
    except KeyboardInterrupt:
        print("收到 Ctrl+C，退出棋盘标定。")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
