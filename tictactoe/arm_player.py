"""井字棋机械臂落子适配层。

本模块只负责一件事：把井字棋格子编号 0..8 转换成机械臂打靶动作需要的
绝对坐标参数。真实舵机通信、IK 和轨迹执行继续复用 tools/arm_control，
这里不重写任何底层机械臂控制。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import ast
import json
import sys
from typing import Any, Sequence


CELL_INDICES = tuple(range(9))


class ArmConfigError(ValueError):
    """井字棋机械臂配置不合法时抛出。"""


@dataclass
class CellTarget:
    """单个井字棋格子的机械臂打点坐标。

    x/y/z_press 是机械臂 base 坐标系下的击打接触点，单位米。
    z_above 是击打前的上方高度，打靶程序会用 z_above - z_press 作为下击高度。
    """

    index: int
    x: float | None = None
    y: float | None = None
    z_above: float | None = None
    z_press: float | None = None
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    label: str = ""

    @classmethod
    def from_dict(cls, index: int, data: dict[str, Any] | None) -> "CellTarget":
        data = data or {}
        return cls(
            index=validate_cell_index(index),
            x=parse_optional_float(data.get("x")),
            y=parse_optional_float(data.get("y")),
            z_above=parse_optional_float(data.get("z_above")),
            z_press=parse_optional_float(data.get("z_press")),
            roll=float(data.get("roll", 0.0) or 0.0),
            pitch=float(data.get("pitch", 0.0) or 0.0),
            yaw=float(data.get("yaw", 0.0) or 0.0),
            label=str(data.get("label", "")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "z_above": self.z_above,
            "z_press": self.z_press,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "label": self.label,
        }

    def missing_fields(self) -> list[str]:
        missing = []
        for name in ("x", "y", "z_press"):
            if getattr(self, name) is None:
                missing.append(name)
        return missing

    def is_configured(self) -> bool:
        return not self.missing_fields()

    def strike_height(self, default_height: float) -> float:
        """计算该格子的下击高度。"""

        if self.z_above is None:
            return float(default_height)
        if self.z_press is None:
            raise ArmConfigError(f"cell {self.index} 缺少 z_press，无法计算 strike_height")
        height = float(self.z_above) - float(self.z_press)
        if height <= 0:
            raise ArmConfigError(
                f"cell {self.index} 的 z_above 必须高于 z_press，当前差值为 {height:.4f} m"
            )
        return height

    def cli_target_args(self, frame: str, default_strike_height: float) -> list[str]:
        """生成 hit_target_action.py 可以使用的参数。"""

        if not self.is_configured():
            raise ArmConfigError(f"cell {self.index} 尚未完整标定，缺少: {', '.join(self.missing_fields())}")
        return [
            "--frame",
            frame,
            "--x",
            f"{float(self.x):.6f}",
            "--y",
            f"{float(self.y):.6f}",
            "--z",
            f"{float(self.z_press):.6f}",
            "--roll",
            f"{float(self.roll):.3f}",
            "--pitch",
            f"{float(self.pitch):.3f}",
            "--yaw",
            f"{float(self.yaw):.3f}",
            "--strike-height",
            f"{self.strike_height(default_strike_height):.6f}",
            "--contact-offset",
            "0.000000",
        ]


@dataclass
class TictactoeArmConfig:
    """井字棋机械臂配置。"""

    port: str = "COM5"
    baudrate: int = 1_000_000
    frame: str = "so101_base"
    hit_config: str = "config/hit_action.json"
    home_config: str = "config/home_pose.json"
    ready_config: str = "config/ready_pose.json"
    default_strike_height: float = 0.12
    dry_run_default: bool = True
    require_execute_flag: bool = True
    cells: dict[int, CellTarget] = field(default_factory=dict)

    @classmethod
    def empty(cls) -> "TictactoeArmConfig":
        config = cls()
        config.cells = {index: CellTarget(index=index) for index in CELL_INDICES}
        return config

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TictactoeArmConfig":
        robot = data.get("robot", {})
        motion = data.get("motion", {})
        safe = data.get("safe", {})
        raw_cells = data.get("cells", {})

        config = cls(
            port=str(robot.get("port", "COM5")),
            baudrate=int(robot.get("baudrate", 1_000_000)),
            frame=str(robot.get("frame", "so101_base")),
            hit_config=str(robot.get("hit_config", "config/hit_action.json")),
            home_config=str(robot.get("home_config", "config/home_pose.json")),
            ready_config=str(robot.get("ready_config", "config/ready_pose.json")),
            default_strike_height=float(motion.get("default_strike_height", 0.12)),
            dry_run_default=bool(safe.get("dry_run_default", True)),
            require_execute_flag=bool(safe.get("require_execute_flag", True)),
        )
        config.cells = {}
        for index in CELL_INDICES:
            cell_data = raw_cells.get(index, raw_cells.get(str(index), {}))
            config.cells[index] = CellTarget.from_dict(index, cell_data)
        return config

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot": {
                "port": self.port,
                "baudrate": self.baudrate,
                "frame": self.frame,
                "hit_config": self.hit_config,
                "home_config": self.home_config,
                "ready_config": self.ready_config,
            },
            "motion": {
                "default_strike_height": self.default_strike_height,
            },
            "safe": {
                "dry_run_default": self.dry_run_default,
                "require_execute_flag": self.require_execute_flag,
            },
            "cells": {index: self.cells[index].to_dict() for index in CELL_INDICES},
        }

    def configured_cells(self) -> list[int]:
        return [index for index, cell in self.cells.items() if cell.is_configured()]

    def missing_cells(self) -> list[int]:
        return [index for index in CELL_INDICES if not self.cells[index].is_configured()]

    def cell(self, index: int) -> CellTarget:
        return self.cells[validate_cell_index(index)]

    def update_cell(self, target: CellTarget) -> None:
        self.cells[validate_cell_index(target.index)] = target


class ArmCellPlayer:
    """把井字棋格子交给现有 SO101 打靶控制器执行。"""

    def __init__(self, config: TictactoeArmConfig, repo_root: str | Path | None = None):
        self.config = config
        self.repo_root = Path(repo_root or Path(__file__).resolve().parents[1])

    def build_hit_command(self, index: int, *, execute: bool = False, port: str | None = None) -> list[str]:
        """生成等价的 hit_target_action.py 命令，便于调试和复制。"""

        cell = self.config.cell(index)
        script = self.repo_root / "tools" / "arm_control" / "hit_target_action.py"
        command = [
            sys.executable,
            str(script),
            "--hit-config",
            self.config.hit_config,
            "--home-config",
            self.config.home_config,
            "--ready-config",
            self.config.ready_config,
            "--port",
            port or self.config.port,
            "--baudrate",
            str(self.config.baudrate),
        ]
        command.extend(cell.cli_target_args(self.config.frame, self.config.default_strike_height))
        if execute:
            command.append("--yes")
        return command

    def plan_or_execute_cell(self, index: int, *, execute: bool = False, port: str | None = None):
        """规划或执行指定格子的打靶动作。

        该函数会连接舵机读取当前姿态；execute=False 时只做 dry-run 规划。
        """

        add_arm_control_paths(self.repo_root)
        from core.arm_controller import ArmController  # type: ignore
        from core.types import Pose6D  # type: ignore

        cell = self.config.cell(index)
        if not cell.is_configured():
            raise ArmConfigError(f"cell {index} 尚未完整标定，缺少: {', '.join(cell.missing_fields())}")

        controller = ArmController.from_files(
            self.config.hit_config,
            home_config_path=self.config.home_config,
            ready_config_path=self.config.ready_config,
            port=port or self.config.port,
            baudrate=self.config.baudrate,
        )
        controller.hit_config["hit_action"]["strike_height_m"] = cell.strike_height(self.config.default_strike_height)
        controller.hit_config["hit_action"]["contact_offset_m"] = 0.0

        target_pose = Pose6D(
            float(cell.x),
            float(cell.y),
            float(cell.z_press),
            float(cell.roll),
            float(cell.pitch),
            float(cell.yaw),
            self.config.frame,
        )

        controller.connect()
        try:
            state = controller.read_state()
            result, target_pose_base = controller.plan_hit(target_pose, state)
            if result.success and execute:
                controller.execute_trajectory(result.trajectory)
            return result, target_pose_base, state
        finally:
            controller.close()


def validate_cell_index(index: int | str) -> int:
    value = int(index)
    if value not in CELL_INDICES:
        raise ArmConfigError(f"格子编号必须是 0..8，当前为 {index}")
    return value


def parse_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in {"", "null", "none", "~"}:
        return None
    return float(value)


def generate_grid_cells(
    *,
    origin_x: float,
    origin_y: float,
    col_dx: float,
    col_dy: float,
    row_dx: float,
    row_dy: float,
    z_press: float,
    z_above: float | None,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> dict[int, CellTarget]:
    """根据左上格中心和行/列方向向量生成 9 个格子。

    origin 表示 cell 0 的中心。
    col 向量表示从左到右移动一格时 x/y 的变化。
    row 向量表示从上到下移动一格时 x/y 的变化。
    """

    cells = {}
    for index in CELL_INDICES:
        row, col = divmod(index, 3)
        cells[index] = CellTarget(
            index=index,
            x=float(origin_x) + col * float(col_dx) + row * float(row_dx),
            y=float(origin_y) + col * float(col_dy) + row * float(row_dy),
            z_press=float(z_press),
            z_above=float(z_above) if z_above is not None else None,
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            label=f"cell_{index}",
        )
    return cells


def add_arm_control_paths(repo_root: str | Path) -> None:
    """让脚本可以复用 tools/arm_control 里的现有控制器。"""

    root = Path(repo_root)
    arm_control = root / "tools" / "arm_control"
    stservo_tools = root / "tools" / "stservo"
    for path in (arm_control, stservo_tools):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def load_arm_config(path: str | Path) -> TictactoeArmConfig:
    config_path = Path(path)
    text = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        data = load_yaml_like(text)
    return TictactoeArmConfig.from_dict(data or {})


def save_arm_config(config: TictactoeArmConfig, path: str | Path) -> None:
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(render_arm_config_yaml(config), encoding="utf-8")


def render_arm_config_yaml(config: TictactoeArmConfig) -> str:
    """写出人能直接改的 YAML。"""

    lines = [
        "# 井字棋机械臂落点配置",
        "# 格子编号与视觉/策略一致：0 1 2 / 3 4 5 / 6 7 8。",
        "robot:",
        f"  port: {config.port}",
        f"  baudrate: {config.baudrate}",
        f"  frame: {config.frame}",
        f"  hit_config: {config.hit_config}",
        f"  home_config: {config.home_config}",
        f"  ready_config: {config.ready_config}",
        "motion:",
        f"  default_strike_height: {config.default_strike_height}",
        "safe:",
        f"  dry_run_default: {str(config.dry_run_default).lower()}",
        f"  require_execute_flag: {str(config.require_execute_flag).lower()}",
        "cells:",
    ]
    for index in CELL_INDICES:
        cell = config.cells.get(index, CellTarget(index=index))
        lines.append(f"  {index}:")
        for key, value in cell.to_dict().items():
            lines.append(f"    {key}: {format_yaml_scalar(value)}")
    return "\n".join(lines) + "\n"


def format_yaml_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def load_yaml_like(text: str) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return load_simple_nested_yaml(text)

    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict):
        raise ArmConfigError("机械臂配置文件顶层必须是字典")
    return data


def load_simple_nested_yaml(text: str) -> dict[str, Any]:
    """极简 YAML 解析器，覆盖本配置文件的缩进字典格式。"""

    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue

        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        if ":" not in stripped:
            raise ArmConfigError(f"无法解析配置行: {raw_line!r}")

        key_text, value_text = stripped.split(":", 1)
        key = parse_yaml_key(key_text.strip())
        value_text = value_text.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]

        if value_text == "":
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = parse_yaml_scalar(value_text)
    return root


def parse_yaml_key(key: str) -> str | int:
    key = key.strip('"').strip("'")
    if key.isdigit():
        return int(key)
    return key


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
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value
