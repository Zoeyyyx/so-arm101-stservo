"""井字棋棋盘视觉识别模块。

第一版目标不是追求“全自动看懂所有场景”，而是提供一条稳定的数据链路：

摄像头/图片 -> 棋盘四角透视矫正 -> 3x3 单元格裁剪 -> HSV 颜色判断 -> 9 格棋盘状态

输出棋盘状态与 strategy.py 保持一致：
- "X"：人类玩家
- "O"：机械臂
- "."：空格
- "?"：视觉不确定，后续 game_manager 应等待下一帧或提示人工检查
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import ast
import json
from typing import Any, Sequence

from .strategy import EMPTY, HUMAN, ROBOT


UNKNOWN = "?"
HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]


@dataclass
class BoardVisionConfig:
    """棋盘视觉配置。

    board_corners 的顺序固定为：左上、右上、右下、左下。
    HSV 阈值现场一定要调，模板值只能作为初始参考。
    """

    camera_index: int = 0
    warp_size: int = 600
    board_corners: list[tuple[float, float] | None] = field(default_factory=lambda: [None, None, None, None])
    human_hsv_lower: tuple[int, int, int] = (0, 80, 80)
    human_hsv_upper: tuple[int, int, int] = (10, 255, 255)
    robot_hsv_lower: tuple[int, int, int] = (0, 0, 0)
    robot_hsv_upper: tuple[int, int, int] = (179, 255, 70)
    human_hsv_ranges: tuple[HsvRange, ...] | None = None
    robot_hsv_ranges: tuple[HsvRange, ...] | None = None
    min_blob_area: int = 2500
    ambiguous_ratio: float = 1.25
    cell_inner_margin_ratio: float = 0.20
    stable_frame_count: int = 5

    @property
    def has_board_corners(self) -> bool:
        return len(self.board_corners) == 4 and all(point is not None for point in self.board_corners)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "BoardVisionConfig":
        corners = data.get("board_corners", [None, None, None, None])
        if corners is None:
            corners = [None, None, None, None]

        human_lower = parse_hsv(data.get("human_hsv_lower", [0, 80, 80]))
        human_upper = parse_hsv(data.get("human_hsv_upper", [10, 255, 255]))
        robot_lower = parse_hsv(data.get("robot_hsv_lower", [0, 0, 0]))
        robot_upper = parse_hsv(data.get("robot_hsv_upper", [179, 255, 70]))

        return cls(
            camera_index=int(data.get("camera_index", 0)),
            warp_size=int(data.get("warp_size", 600)),
            board_corners=[parse_point(point) for point in corners],
            human_hsv_lower=human_lower,
            human_hsv_upper=human_upper,
            robot_hsv_lower=robot_lower,
            robot_hsv_upper=robot_upper,
            human_hsv_ranges=parse_hsv_ranges(data.get("human_hsv_ranges"), human_lower, human_upper),
            robot_hsv_ranges=parse_hsv_ranges(data.get("robot_hsv_ranges"), robot_lower, robot_upper),
            min_blob_area=int(data.get("min_blob_area", 2500)),
            ambiguous_ratio=float(data.get("ambiguous_ratio", 1.25)),
            cell_inner_margin_ratio=float(data.get("cell_inner_margin_ratio", 0.20)),
            stable_frame_count=int(data.get("stable_frame_count", 5)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_index": self.camera_index,
            "warp_size": self.warp_size,
            "board_corners": [list(point) if point is not None else None for point in self.board_corners],
            "human_hsv_lower": list(self.human_hsv_lower),
            "human_hsv_upper": list(self.human_hsv_upper),
            "robot_hsv_lower": list(self.robot_hsv_lower),
            "robot_hsv_upper": list(self.robot_hsv_upper),
            "human_hsv_ranges": [
                [list(lower), list(upper)] for lower, upper in self.effective_human_hsv_ranges()
            ],
            "robot_hsv_ranges": [
                [list(lower), list(upper)] for lower, upper in self.effective_robot_hsv_ranges()
            ],
            "min_blob_area": self.min_blob_area,
            "ambiguous_ratio": self.ambiguous_ratio,
            "cell_inner_margin_ratio": self.cell_inner_margin_ratio,
            "stable_frame_count": self.stable_frame_count,
        }

    def effective_human_hsv_ranges(self) -> tuple[HsvRange, ...]:
        if self.human_hsv_ranges:
            return self.human_hsv_ranges
        return ((self.human_hsv_lower, self.human_hsv_upper),)

    def effective_robot_hsv_ranges(self) -> tuple[HsvRange, ...]:
        if self.robot_hsv_ranges:
            return self.robot_hsv_ranges
        return ((self.robot_hsv_lower, self.robot_hsv_upper),)


@dataclass(frozen=True)
class CellVisionResult:
    """单个格子的识别结果。"""

    index: int
    state: str
    human_pixels: int
    robot_pixels: int
    center_xy: tuple[float, float]


@dataclass(frozen=True)
class BoardVisionResult:
    """一帧棋盘识别结果。"""

    board: tuple[str, ...]
    cells: tuple[CellVisionResult, ...]
    stable_board: tuple[str, ...] | None = None


class BoardStabilityFilter:
    """连续多帧一致后再确认棋盘，降低偶发误判。"""

    def __init__(self, required_count: int = 5):
        self.required_count = max(1, int(required_count))
        self.last_board: tuple[str, ...] | None = None
        self.same_count = 0
        self.stable_board: tuple[str, ...] | None = None

    def update(self, board: Sequence[str]) -> tuple[str, ...] | None:
        current = tuple(board)
        if current == self.last_board:
            self.same_count += 1
        else:
            self.last_board = current
            self.same_count = 1

        if self.same_count >= self.required_count:
            self.stable_board = current
        return self.stable_board

    def reset(self) -> None:
        self.last_board = None
        self.same_count = 0
        self.stable_board = None


def parse_hsv(value: Sequence[Any]) -> tuple[int, int, int]:
    if len(value) != 3:
        raise ValueError(f"HSV 阈值必须包含 3 个数，当前为 {value!r}")
    return tuple(int(item) for item in value)  # type: ignore[return-value]


def parse_hsv_ranges(value: Any, fallback_lower: Sequence[Any], fallback_upper: Sequence[Any]) -> tuple[HsvRange, ...]:
    """解析一个或多个 HSV 区间。

    红色在 OpenCV HSV 里会跨 0/179 边界，因此支持多个区间。
    配置推荐写法：
      - [[0, 90, 80], [10, 255, 255]]
      - [[170, 90, 80], [179, 255, 255]]
    """

    if value is None:
        return ((parse_hsv(fallback_lower), parse_hsv(fallback_upper)),)

    ranges: list[HsvRange] = []
    for item in value:
        if isinstance(item, dict):
            lower = item.get("lower")
            upper = item.get("upper")
        else:
            if len(item) != 2:
                raise ValueError(f"HSV 区间必须是 [lower, upper]，当前为 {item!r}")
            lower, upper = item
        ranges.append((parse_hsv(lower), parse_hsv(upper)))

    if not ranges:
        raise ValueError("HSV 区间列表不能为空")
    return tuple(ranges)


def parse_point(value: Any) -> tuple[float, float] | None:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in {"", "null", "none"}:
        return None
    if len(value) != 2:
        raise ValueError(f"棋盘角点必须是 [x, y]，当前为 {value!r}")
    return float(value[0]), float(value[1])


def classify_cell_from_scores(
    human_pixels: int,
    robot_pixels: int,
    *,
    min_blob_area: int,
    ambiguous_ratio: float = 1.25,
) -> str:
    """根据颜色像素数量判断一个格子的状态。

    两种颜色都很少：空格。
    两种颜色接近：不确定，避免机械臂误落子。
    """

    human_pixels = int(human_pixels)
    robot_pixels = int(robot_pixels)
    if max(human_pixels, robot_pixels) < min_blob_area:
        return EMPTY
    if human_pixels >= robot_pixels * ambiguous_ratio:
        return HUMAN
    if robot_pixels >= human_pixels * ambiguous_ratio:
        return ROBOT
    return UNKNOWN


def cell_center(index: int, warp_size: int) -> tuple[float, float]:
    """返回透视矫正后第 index 格的中心点像素坐标。"""

    if index not in range(9):
        raise ValueError(f"格子编号必须是 0..8，当前为 {index}")
    cell = warp_size / 3.0
    row, col = divmod(index, 3)
    return (col + 0.5) * cell, (row + 0.5) * cell


def format_vision_board(board: Sequence[str]) -> str:
    """把视觉输出格式化为三行，允许 ?。"""

    cells = tuple(board)
    if len(cells) != 9:
        raise ValueError(f"棋盘必须正好包含 9 个格子，当前为 {len(cells)} 个")
    return "\n".join(" ".join(cells[row : row + 3]) for row in range(0, 9, 3))


def require_cv2():
    """延迟导入 OpenCV，避免纯策略测试被视觉依赖卡住。"""

    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "视觉功能需要安装 opencv-python 和 numpy。"
            "建议在当前 conda 环境执行：python -m pip install opencv-python pyyaml"
        ) from exc
    return cv2, np


def mask_hsv_ranges(cv2: Any, np: Any, roi_hsv: Any, ranges: Sequence[HsvRange]) -> Any:
    """把多个 HSV 区间合并成一个 mask。"""

    mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        lower_array = np.array(lower, dtype=np.uint8)
        upper_array = np.array(upper, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(roi_hsv, lower_array, upper_array))
    return mask


def warp_board(frame: Any, config: BoardVisionConfig) -> Any:
    """根据四角点把棋盘透视矫正为正方形图像。"""

    if not config.has_board_corners:
        raise ValueError("尚未配置 board_corners，请先运行 scripts/calibrate_board_vision.py")

    cv2, np = require_cv2()
    src = np.array(config.board_corners, dtype=np.float32)
    size = float(config.warp_size)
    dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (config.warp_size, config.warp_size))


def analyze_warped_board(warped_bgr: Any, config: BoardVisionConfig) -> BoardVisionResult:
    """识别已经透视矫正后的 3x3 棋盘。"""

    cv2, np = require_cv2()
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    cell_size = config.warp_size // 3
    margin = int(cell_size * config.cell_inner_margin_ratio)
    human_ranges = config.effective_human_hsv_ranges()
    robot_ranges = config.effective_robot_hsv_ranges()

    results: list[CellVisionResult] = []
    for index in range(9):
        row, col = divmod(index, 3)
        x0 = col * cell_size + margin
        y0 = row * cell_size + margin
        x1 = (col + 1) * cell_size - margin
        y1 = (row + 1) * cell_size - margin
        roi = hsv[y0:y1, x0:x1]

        human_mask = mask_hsv_ranges(cv2, np, roi, human_ranges)
        robot_mask = mask_hsv_ranges(cv2, np, roi, robot_ranges)
        human_pixels = int(cv2.countNonZero(human_mask))
        robot_pixels = int(cv2.countNonZero(robot_mask))
        state = classify_cell_from_scores(
            human_pixels,
            robot_pixels,
            min_blob_area=config.min_blob_area,
            ambiguous_ratio=config.ambiguous_ratio,
        )
        results.append(
            CellVisionResult(
                index=index,
                state=state,
                human_pixels=human_pixels,
                robot_pixels=robot_pixels,
                center_xy=cell_center(index, config.warp_size),
            )
        )

    return BoardVisionResult(board=tuple(item.state for item in results), cells=tuple(results))


def recognize_frame(frame_bgr: Any, config: BoardVisionConfig) -> BoardVisionResult:
    """从原始相机帧识别棋盘状态。"""

    warped = warp_board(frame_bgr, config)
    return analyze_warped_board(warped, config)


def draw_debug_overlay(warped_bgr: Any, result: BoardVisionResult, config: BoardVisionConfig) -> Any:
    """在矫正图上画九宫格和识别结果，供调试窗口显示。"""

    cv2, _ = require_cv2()
    image = warped_bgr.copy()
    cell_size = config.warp_size // 3
    for i in range(1, 3):
        cv2.line(image, (i * cell_size, 0), (i * cell_size, config.warp_size), (255, 255, 255), 2)
        cv2.line(image, (0, i * cell_size), (config.warp_size, i * cell_size), (255, 255, 255), 2)

    for cell in result.cells:
        x, y = cell.center_xy
        color = (0, 255, 0) if cell.state != UNKNOWN else (0, 255, 255)
        cv2.putText(image, cell.state, (int(x) - 15, int(y) + 15), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        cv2.putText(
            image,
            f"H:{cell.human_pixels} R:{cell.robot_pixels}",
            (int(x) - 55, int(y) + 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
        )
    return image


def load_vision_config(path: str | Path) -> BoardVisionConfig:
    """读取视觉配置，优先支持 YAML，也兼容 JSON。"""

    config_path = Path(path)
    text = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        data = load_yaml_like(text)
    return BoardVisionConfig.from_dict(data)


def save_vision_config(config: BoardVisionConfig, path: str | Path) -> None:
    """写出视觉配置。使用手写 YAML，避免强制依赖 PyYAML。"""

    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    data = config.to_dict()
    lines = [
        "# 井字棋视觉配置",
        "# board_corners 顺序：左上、右上、右下、左下。",
        f"camera_index: {data['camera_index']}",
        f"warp_size: {data['warp_size']}",
        "board_corners:",
    ]
    for point in data["board_corners"]:
        lines.append(f"  - {json.dumps(point, ensure_ascii=False) if point is not None else 'null'}")

    for key in (
        "human_hsv_lower",
        "human_hsv_upper",
        "robot_hsv_lower",
        "robot_hsv_upper",
        "human_hsv_ranges",
        "robot_hsv_ranges",
        "min_blob_area",
        "ambiguous_ratio",
        "cell_inner_margin_ratio",
        "stable_frame_count",
    ):
        value = data[key]
        rendered = json.dumps(value, ensure_ascii=False) if isinstance(value, list) else value
        lines.append(f"{key}: {rendered}")

    config_path.write_text("\n".join(str(line) for line in lines) + "\n", encoding="utf-8")


def load_yaml_like(text: str) -> dict[str, Any]:
    """读取本项目用到的简化 YAML。

    如果安装了 PyYAML，则使用 safe_load；否则用一个很小的解析器处理模板配置。
    """

    try:
        import yaml  # type: ignore
    except ImportError:
        return load_simple_yaml(text)

    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict):
        raise ValueError("视觉配置文件顶层必须是字典")
    return data


def load_simple_yaml(text: str) -> dict[str, Any]:
    """极简 YAML 解析器，只覆盖本配置文件需要的 key/value 和列表。"""

    data: dict[str, Any] = {}
    active_list_key: str | None = None
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue

        stripped = line.strip()
        if stripped.startswith("- "):
            if active_list_key is None:
                raise ValueError(f"列表项没有对应 key: {raw_line!r}")
            data.setdefault(active_list_key, []).append(parse_yaml_scalar(stripped[2:].strip()))
            continue

        if ":" not in stripped:
            raise ValueError(f"无法解析配置行: {raw_line!r}")
        key, value = stripped.split(":", 1)
        key = key.strip()
        value = value.strip()
        if value == "":
            data[key] = []
            active_list_key = key
        else:
            data[key] = parse_yaml_scalar(value)
            active_list_key = None
    return data


def parse_yaml_scalar(value: str) -> Any:
    lower = value.lower()
    if lower in {"null", "none", "~"}:
        return None
    if lower == "true":
        return True
    if lower == "false":
        return False
    if value.startswith("[") or value.startswith("{"):
        return ast.literal_eval(value.replace("null", "None").replace("true", "True").replace("false", "False"))
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value.strip('"').strip("'")
