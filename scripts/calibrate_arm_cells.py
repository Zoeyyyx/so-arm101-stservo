"""Calibrate arm target coordinates for the 9 tic-tac-toe cells.

Default mode previews changes only. Add --yes to save.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.arm_player import (  # noqa: E402
    ArmCellPlayer,
    CellTarget,
    TictactoeArmConfig,
    generate_grid_cells,
    load_arm_config,
    save_arm_config,
    validate_cell_index,
)


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "tictactoe_arm.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Calibrate arm base coordinates for tic-tac-toe cells.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Tic-tac-toe arm config path.")
    parser.add_argument("--output", help="Output config path. Defaults to --config.")
    parser.add_argument("--print", action="store_true", help="Print current calibration only.")

    parser.add_argument("--cell", type=int, help="Cell index to set, 0..8.")
    parser.add_argument("--x", type=float, help="Cell press target x, meters.")
    parser.add_argument("--y", type=float, help="Cell press target y, meters.")
    parser.add_argument("--z-press", type=float, help="Press/contact z, meters.")
    parser.add_argument("--z-above", type=float, help="Above/approach z, meters.")
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--label", default="")

    parser.add_argument("--generate-grid", action="store_true", help="Generate all 9 cells from origin and row/column vectors.")
    parser.add_argument("--origin-x", type=float, help="Cell 0 center x.")
    parser.add_argument("--origin-y", type=float, help="Cell 0 center y.")
    parser.add_argument("--col-dx", type=float, help="X change when moving one cell left-to-right.")
    parser.add_argument("--col-dy", type=float, help="Y change when moving one cell left-to-right.")
    parser.add_argument("--row-dx", type=float, help="X change when moving one cell top-to-bottom.")
    parser.add_argument("--row-dy", type=float, help="Y change when moving one cell top-to-bottom.")

    parser.add_argument("--from-current", action="store_true", help="Connect the arm and read the current end-effector FK position.")
    parser.add_argument(
        "--current-as",
        choices=["press", "above"],
        default="press",
        help="Save the current end-effector z as z_press or z_above.",
    )
    parser.add_argument("--port", help="Override the configured COM port, e.g. COM5.")
    parser.add_argument("--baudrate", type=int, help="Override the configured baudrate.")
    parser.add_argument("--yes", action="store_true", help="Confirm writing the config file.")
    return parser


def print_config(config: TictactoeArmConfig) -> None:
    print("Tic-tac-toe arm target config:")
    print(f"  port={config.port} baudrate={config.baudrate} frame={config.frame}")
    print(f"  hit_config={config.hit_config}")
    print(f"  home_config={config.home_config}")
    print(f"  ready_config={config.ready_config}")
    print(f"  default_strike_height={config.default_strike_height:.4f} m")
    print("Cell calibration:")
    for index in range(9):
        cell = config.cells[index]
        mark = "OK" if cell.is_configured() else "MISSING"
        missing = "" if cell.is_configured() else f" missing={','.join(cell.missing_fields())}"
        print(
            f"  cell {index}: {mark:7s} "
            f"x={format_value(cell.x)} y={format_value(cell.y)} "
            f"z_above={format_value(cell.z_above)} z_press={format_value(cell.z_press)} "
            f"rpy=({cell.roll:.1f},{cell.pitch:.1f},{cell.yaw:.1f}) "
            f"label={cell.label!r}{missing}"
        )


def format_value(value: float | None) -> str:
    return "null" if value is None else f"{value:.4f}"


def require_args(args: argparse.Namespace, names: list[str]) -> None:
    missing = [name for name in names if getattr(args, name.replace("-", "_")) is None]
    if missing:
        raise SystemExit(f"Missing arguments: {', '.join('--' + name for name in missing)}")


def apply_manual_cell(config: TictactoeArmConfig, args: argparse.Namespace) -> None:
    require_args(args, ["cell", "x", "y", "z-press"])
    index = validate_cell_index(args.cell)
    previous = config.cells[index]
    config.update_cell(
        CellTarget(
            index=index,
            x=args.x,
            y=args.y,
            z_above=args.z_above,
            z_press=args.z_press,
            roll=args.roll,
            pitch=args.pitch,
            yaw=args.yaw,
            label=args.label or previous.label or f"cell_{index}",
        )
    )


def apply_generated_grid(config: TictactoeArmConfig, args: argparse.Namespace) -> None:
    require_args(args, ["origin-x", "origin-y", "col-dx", "col-dy", "row-dx", "row-dy", "z-press"])
    config.cells = generate_grid_cells(
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        col_dx=args.col_dx,
        col_dy=args.col_dy,
        row_dx=args.row_dx,
        row_dy=args.row_dy,
        z_press=args.z_press,
        z_above=args.z_above,
        roll=args.roll,
        pitch=args.pitch,
        yaw=args.yaw,
    )


def apply_current_pose(config: TictactoeArmConfig, args: argparse.Namespace) -> None:
    require_args(args, ["cell"])
    index = validate_cell_index(args.cell)
    if args.port:
        config.port = args.port
    if args.baudrate:
        config.baudrate = args.baudrate

    player = ArmCellPlayer(config, repo_root=PROJECT_ROOT)
    player_module_root = player.repo_root
    from tictactoe.arm_player import add_arm_control_paths  # Deferred path setup.

    add_arm_control_paths(player_module_root)
    from core.arm_controller import ArmController  # type: ignore
    from core.types import Pose6D  # type: ignore

    controller = ArmController.from_files(
        config.hit_config,
        home_config_path=config.home_config,
        ready_config_path=config.ready_config,
        port=config.port,
        baudrate=config.baudrate,
    )
    controller.connect()
    try:
        state = controller.read_state()
        template = Pose6D(0.0, 0.0, 0.0, args.roll, args.pitch, args.yaw, config.frame)
        pose = controller.ik_solver.fk_pose(state.angles, template)
    finally:
        controller.close()

    previous = config.cells[index]
    z_above = previous.z_above
    z_press = previous.z_press
    if args.current_as == "above":
        z_above = pose.z
    else:
        z_press = pose.z

    config.update_cell(
        CellTarget(
            index=index,
            x=pose.x,
            y=pose.y,
            z_above=z_above,
            z_press=z_press,
            roll=args.roll,
            pitch=args.pitch,
            yaw=args.yaw,
            label=args.label or previous.label or f"cell_{index}",
        )
    )
    print(
        f"Read current end-effector FK: cell={index} x={pose.x:.4f} y={pose.y:.4f} z={pose.z:.4f} "
        f"saved_as={args.current_as}"
    )


def main() -> None:
    args = build_parser().parse_args()
    config = load_arm_config(args.config)

    changed = False
    if args.generate_grid:
        apply_generated_grid(config, args)
        changed = True
    elif args.from_current:
        apply_current_pose(config, args)
        changed = True
    elif args.cell is not None or any(getattr(args, name) is not None for name in ("x", "y", "z_press", "z_above")):
        apply_manual_cell(config, args)
        changed = True

    print_config(config)

    if not changed or args.print:
        return

    output = args.output or args.config
    if not args.yes:
        print("Preview only. No config was written. Add --yes to save.")
        return

    save_arm_config(config, output)
    print(f"Saved tic-tac-toe arm target config: {output}")


if __name__ == "__main__":
    main()
