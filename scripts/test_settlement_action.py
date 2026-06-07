"""单独测试井字棋结算动作。

默认只预览动作配置，不连接机械臂；加 --execute 或 --yes 才真实执行。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tictactoe.arm_player import load_arm_config  # noqa: E402
from tictactoe.settlement_actions import DEFAULT_ACTION_CONFIG, SettlementActionPlayer  # noqa: E402


DEFAULT_ARM_CONFIG = PROJECT_ROOT / "config" / "tictactoe_arm.yaml"

ACTION_TO_OUTCOME = {
    "victory": "robot_win",
    "draw_handshake": "draw",
    "defeat_nod": "human_win",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="单独测试井字棋对局结算动作")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--outcome", choices=["robot_win", "draw", "human_win"], help="按对局结果选择动作")
    group.add_argument("--action", choices=["victory", "draw_handshake", "defeat_nod"], help="直接选择动作名")
    parser.add_argument("--arm-config", default=str(DEFAULT_ARM_CONFIG), help="井字棋机械臂配置")
    parser.add_argument("--settlement-config", default=str(DEFAULT_ACTION_CONFIG), help="结算动作配置")
    parser.add_argument("--port", help="覆盖 COM 口，例如 COM5")
    parser.add_argument("--execute", action="store_true", help="真实执行动作")
    parser.add_argument("--yes", action="store_true", help="真实执行动作，等价于 --execute")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    outcome = args.outcome or ACTION_TO_OUTCOME[args.action]
    execute = bool(args.execute or args.yes)

    arm_config = load_arm_config(args.arm_config)
    if args.port:
        arm_config.port = args.port

    print(f"结算动作测试: outcome={outcome} execute={execute}")
    player = SettlementActionPlayer(
        arm_config,
        settlement_config_path=args.settlement_config,
        repo_root=PROJECT_ROOT,
    )
    player.play(outcome, execute=execute, port=args.port)

    if not execute:
        print("当前只是预览。确认安全后追加 --execute 或 --yes 才会真实发送舵机指令。")


if __name__ == "__main__":
    main()
