"""通过 LeRobot 向 SO101 follower 发送关节目标。

默认只预览目标，不真正调用 `robot.send_action()`；确认安全后需要显式加
`--yes`。后续 ROS arm driver 节点可以复用这个脚本里的动作组织方式。
"""

import argparse
import os
import time

# Windows 上 conda-forge/torch 可能同时带入两套 OpenMP runtime。
# 这里先设置环境变量，并且先导入 torch，再导入 LeRobot，避免 DLL 加载顺序问题。
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import torch  # noqa: F401

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SOFollower


JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def parse_joint_value(items):
    action = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"关节目标必须写成 name=value，例如 wrist_flex=10。收到: {item}")
        name, value = item.split("=", 1)
        name = name.strip()
        if name not in JOINTS:
            raise ValueError(f"未知关节: {name}。可用关节: {', '.join(JOINTS)}")
        action[f"{name}.pos"] = float(value)
    return action


def build_action(requested_action, observation, use_relative):
    if not use_relative:
        return requested_action

    action = {}
    for key, delta in requested_action.items():
        if key not in observation:
            raise KeyError(f"当前 observation 里没有 {key}")
        # relative 模式下，命令值表示“相对当前位置移动多少”，不是绝对目标。
        action[key] = float(observation[key]) + float(delta)
    return action


def main():
    parser = argparse.ArgumentParser(
        description="LeRobot 关节动作发送工具：发送一个或多个关节目标位置。"
    )
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--id", default="soarm101_follower")
    parser.add_argument(
        "--set",
        nargs="+",
        required=True,
        help="目标关节位置，例如 --set shoulder_pan=0 wrist_flex=10 gripper=30",
    )
    parser.add_argument(
        "--relative",
        action="store_true",
        help="把 --set 里的值解释为相对当前位置的变化量，而不是绝对目标位置。",
    )
    parser.add_argument(
        "--max-relative",
        type=float,
        default=10.0,
        help="单次动作相对当前姿态的最大变化量，默认 10。用于降低误操作风险。",
    )
    parser.add_argument("--hold", type=float, default=0.8)
    parser.add_argument("--yes", action="store_true", help="真正发送动作；不加时只预览。")
    args = parser.parse_args()

    requested_action = parse_joint_value(args.set)

    # max_relative_target 是 LeRobot 自带的动作保护：目标离当前位置太远时会被裁剪。
    config = SO101FollowerConfig(
        port=args.port,
        id=args.id,
        max_relative_target=args.max_relative,
    )
    robot = SOFollower(config)

    try:
        robot.connect()
        obs = robot.get_observation()
        print("当前关节位置:")
        for joint in JOINTS:
            key = f"{joint}.pos"
            if key in obs:
                print(f"  {key}: {float(obs[key]):.3f}")

        action = build_action(requested_action, obs, args.relative)

        print("命令行请求值:")
        for key, value in requested_action.items():
            mode = "相对偏移" if args.relative else "绝对目标"
            print(f"  {key} {mode}: {value:.3f}")

        print("实际将发送的 action:")
        for key, value in action.items():
            print(f"  {key}: {value:.3f}")

        if not args.yes:
            print("当前只是预览。确认安全后追加 --yes，才会调用 robot.send_action()。")
            return

        sent = robot.send_action(action)
        print("LeRobot 安全裁剪后的实际发送值:")
        for key, value in sent.items():
            print(f"  {key}: {float(value):.3f}")

        time.sleep(args.hold)
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
