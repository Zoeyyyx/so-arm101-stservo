"""SO101 低速平滑回 home CLI。

这个脚本专门负责：
当前姿态 -> config/home_pose.json 中保存的 home/stow。

打靶脚本 hit_target_action.py 不再自动从任意姿态回 home；若当前位置不在
home 附近，应先运行本脚本。
"""

from __future__ import annotations

import argparse
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"
ALL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def build_parser():
    """创建 CLI 参数。"""
    parser = argparse.ArgumentParser(description="低速平滑回到 SO101 home/stow。")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG), help="打靶动作配置。")
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG), help="home/stow 姿态配置。")
    parser.add_argument("--port", help="覆盖 COM 口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖波特率。")
    parser.add_argument("--speed", type=int, default=60, help="回 home 速度，默认较低。")
    parser.add_argument("--acc", type=int, default=6, help="回 home 加速度，默认较低。")
    parser.add_argument("--steps", type=int, default=90, help="回 home 插值步数。")
    parser.add_argument("--dt", type=float, default=0.06, help="每步延时，单位秒。")
    parser.add_argument("--yes", action="store_true", help="真正执行；不加时只 dry-run。")
    return parser


def print_state_and_home(state, home_angles):
    """打印当前姿态和 home 差异。"""
    print("当前姿态 -> home:")
    for joint in ALL_JOINTS:
        delta = float(home_angles[joint]) - float(state.angles[joint])
        print(
            f"  {joint:14s} current={state.angles[joint]:8.3f} "
            f"home={home_angles[joint]:8.3f} delta={delta:+8.3f} raw={state.raw[joint]}"
        )


def print_plan_summary(result):
    """打印回 home 轨迹摘要。"""
    from core.trajectory_planner import max_adjacent_joint_delta, max_joint_delta_degrees

    if not result.trajectory:
        print("没有生成轨迹。")
        return
    first = result.trajectory[0]
    last = result.trajectory[-1]
    print("return_home 轨迹概要:")
    print(
        f"  points={len(result.trajectory)} speed={last.speed} acc={last.acc} dt={last.dt:.3f} "
        f"duration={sum(point.dt for point in result.trajectory):.2f}s "
        f"max_step_delta={max_adjacent_joint_delta(result.trajectory):.3f}deg "
        f"total_delta={max_joint_delta_degrees(first.angles, last.angles):.3f}deg"
    )
    print("最终目标 raw:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} raw={last.raw[joint]}")


def main():
    args = build_parser().parse_args()
    from core.arm_controller import ArmController

    controller = ArmController.from_files(
        args.hit_config,
        home_config_path=args.home_config,
        port=args.port,
        baudrate=args.baudrate,
    )

    controller.connect()
    try:
        state = controller.read_state()
        home_angles = controller.planner.home_angles(
            state.angles,
            controller.hit_config["hit_action"].get("default_gripper", state.angles["gripper"]),
        )
        result = controller.plan_return_home(
            state,
            speed=args.speed,
            acc=args.acc,
            steps=args.steps,
            dt=args.dt,
        )

        print("配置文件:")
        print(f"  home_pose={controller.home_pose.get('_path')}")
        print(f"  serial={controller.controller_config['serial']['port']} baudrate={controller.controller_config['serial']['baudrate']}")
        print_state_and_home(state, home_angles)
        print_plan_summary(result)

        if not args.yes:
            print("当前只是 dry-run。确认回 home 路径安全后追加 --yes 才会执行。")
            return

        print("开始低速回 home。")
        controller.execute_trajectory(result.trajectory)
        print("已回到 home。")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
