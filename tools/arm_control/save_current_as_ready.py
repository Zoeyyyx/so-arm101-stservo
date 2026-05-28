"""读取当前机械臂姿态，并保存为打靶程序的 ready 安全展开姿态。

使用场景：
1. 先手动把 SO101 机械臂摆到安全展开姿态；
2. 运行本脚本读取六个舵机当前位置；
3. 确认角度合理后加 --yes 写入 config/ready_pose.json。

注意：
- ready_pose 用于 home -> ready 的关节空间展开；
- 它不是精确末端坐标点，只是一个安全、平滑、可执行的中间关节姿态；
- 本脚本只读取位置和写配置，不发送舵机运动指令。
"""

import argparse
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
ARM_CONTROL_DIR = REPO_ROOT / "tools" / "arm_control"
if str(ARM_CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(ARM_CONTROL_DIR))

ALL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_READY_CONFIG = REPO_ROOT / "config" / "ready_pose.json"


def resolve_path(path):
    """把相对路径转换为项目内绝对路径。"""
    result = Path(path)
    if not result.is_absolute():
        result = REPO_ROOT / result
    return result


def load_hit_config(path):
    """读取打靶配置 JSON。"""
    config_path = resolve_path(path)
    return config_path, json.loads(config_path.read_text(encoding="utf-8"))


def resolve_controller_config(hit_config):
    """读取底层舵机/IK 控制配置。"""
    from send_absolute_pose_template import load_config

    controller_config = resolve_path(hit_config["controller_config"])
    return load_config(controller_config)


def rounded_angles(current_angles, digits):
    """把读取到的角度四舍五入，避免配置文件里出现过长小数。"""
    return {joint: round(float(current_angles[joint]), int(digits)) for joint in ALL_JOINTS}


def main():
    parser = argparse.ArgumentParser(description="把当前机械臂姿态保存为打靶 ready 安全展开姿态。")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG), help="打靶动作配置文件。")
    parser.add_argument("--ready-config", default=str(DEFAULT_READY_CONFIG), help="ready 姿态配置文件。")
    parser.add_argument("--port", help="覆盖配置中的 COM 口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖配置中的波特率。")
    parser.add_argument("--digits", type=int, default=3, help="保存角度的小数位数。")
    parser.add_argument("--yes", action="store_true", help="确认写入配置文件；不加时只打印。")
    args = parser.parse_args()
    from send_absolute_pose_template import open_bus, read_current_joint_angles

    hit_config_path, hit_config = load_hit_config(args.hit_config)
    ready_config_path = resolve_path(args.ready_config)
    controller_config = resolve_controller_config(hit_config)
    if args.port:
        controller_config["serial"]["port"] = args.port
    if args.baudrate:
        controller_config["serial"]["baudrate"] = args.baudrate

    port_handler, packet_handler = open_bus(
        controller_config["serial"]["port"],
        int(controller_config["serial"]["baudrate"]),
    )
    try:
        current_angles, current_raw = read_current_joint_angles(packet_handler, controller_config)
    finally:
        port_handler.closePort()

    ready_angles = rounded_angles(current_angles, args.digits)

    print("当前 raw 位置:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} raw={current_raw[joint]}")
    print("准备保存为 ready 的角度，单位 deg:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} {ready_angles[joint]:8.3f}")

    if not args.yes:
        print("当前只是 dry-run。确认这个展开姿态安全后，追加 --yes 写入 ready_pose.json。")
        return

    ready_pose = {
        "enabled": True,
        "joint_angles_deg": ready_angles,
        "note": (
            "SO101 打靶前的安全展开姿态，单位度。"
            "home -> ready_pose 使用关节空间插值，不做笛卡尔直线 IK。"
        ),
    }
    ready_config_path.write_text(
        json.dumps(ready_pose, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    hit_config["ready_pose_config"] = str(ready_config_path.relative_to(REPO_ROOT)).replace("\\", "/")
    hit_config_path.write_text(
        json.dumps(hit_config, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"已写入 ready 姿态: {ready_config_path}")
    print(f"已更新打靶配置: {hit_config_path}")


if __name__ == "__main__":
    main()
