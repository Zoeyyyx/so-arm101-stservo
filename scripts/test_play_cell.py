"""测试机械臂在指定井字棋格子落子。

默认不会连接机械臂，只打印将要使用的打靶命令。
加 --plan 会连接机械臂做 dry-run 规划。
加 --execute 才会真实执行动作。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.arm_player import ArmCellPlayer, ArmConfigError, load_arm_config, validate_cell_index  # noqa: E402


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "tictactoe_arm.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="测试井字棋某一格的机械臂落子动作")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="井字棋机械臂配置路径")
    parser.add_argument("--cell", type=int, required=True, help="格子编号 0..8")
    parser.add_argument("--port", help="覆盖配置里的 COM 口，例如 COM5")
    parser.add_argument("--plan", action="store_true", help="连接机械臂并做 dry-run 规划")
    parser.add_argument("--execute", action="store_true", help="真实执行动作")
    return parser


def print_cell_info(player: ArmCellPlayer, cell_index: int) -> None:
    cell = player.config.cell(cell_index)
    print(f"目标格子: cell {cell.index} label={cell.label!r}")
    print(
        f"  x={cell.x} y={cell.y} z_press={cell.z_press} z_above={cell.z_above} "
        f"roll={cell.roll} pitch={cell.pitch} yaw={cell.yaw}"
    )
    print(f"  strike_height={cell.strike_height(player.config.default_strike_height):.4f} m")


def print_plan_summary(result, target_pose_base) -> None:
    print("规划结果:")
    print(f"  success={result.success}")
    if result.reason:
        print(f"  reason={result.reason}")
    print(
        "  base目标: "
        f"x={target_pose_base.x:.4f} y={target_pose_base.y:.4f} z={target_pose_base.z:.4f} "
        f"frame={target_pose_base.frame}"
    )
    if result.trajectory:
        phases = []
        for point in result.trajectory:
            if point.phase not in phases:
                phases.append(point.phase)
        print(f"  trajectory_points={len(result.trajectory)}")
        print(f"  phases={', '.join(phases)}")


def main() -> None:
    args = build_parser().parse_args()
    cell_index = validate_cell_index(args.cell)
    config = load_arm_config(args.config)
    player = ArmCellPlayer(config, repo_root=PROJECT_ROOT)

    try:
        print_cell_info(player, cell_index)
        command = player.build_hit_command(cell_index, execute=args.execute, port=args.port)
    except ArmConfigError as exc:
        raise SystemExit(str(exc)) from exc

    print("等价 hit_target_action.py 命令:")
    print(" ".join(shlex.quote(part) for part in command))

    if not args.plan and not args.execute:
        print("当前未连接机械臂。需要 dry-run 规划时加 --plan，需要真实执行时加 --execute。")
        return

    result, target_pose_base, _state = player.plan_or_execute_cell(cell_index, execute=args.execute, port=args.port)
    print_plan_summary(result, target_pose_base)
    if args.execute:
        print("单格落子动作执行完成。")
    else:
        print("当前只是 dry-run 规划，未发送动作。")


if __name__ == "__main__":
    main()
