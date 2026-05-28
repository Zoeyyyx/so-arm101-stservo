"""读取当前机械臂姿态，并保存为打靶程序的 home 收起位姿。

使用场景：
1. 先手动把 SO101 机械臂摆到你认为安全的收起姿态；
2. 运行本脚本读取六个舵机当前位置；
3. 确认角度合理后加 --yes 写入 config/hit_action.json。

注意：
- 默认只打印，不写文件；
- 本脚本只保存 home_pose，不会发送舵机运动指令；
- 读取角度依赖 config/absolute_pose_control.json 中的 raw_min/raw_max 映射。
"""

import argparse
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
ARM_CONTROL_DIR = REPO_ROOT / "tools" / "arm_control"
if str(ARM_CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(ARM_CONTROL_DIR))

from send_absolute_pose_template import (  # noqa: E402
    ALL_JOINTS,
    load_config,
    open_bus,
    read_current_joint_angles,
)


DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"


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
    controller_config = resolve_path(hit_config["controller_config"])
    return load_config(controller_config)


def rounded_angles(current_angles, digits):
    """把读取到的角度四舍五入，避免配置文件里出现过长小数。"""
    return {joint: round(float(current_angles[joint]), int(digits)) for joint in ALL_JOINTS}


def main():
    parser = argparse.ArgumentParser(description="把当前机械臂姿态保存为打靶 home 收起位姿。")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG), help="打靶动作配置文件。")
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG), help="独立 home 姿态配置文件。")
    parser.add_argument("--port", help="覆盖配置中的 COM 口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖配置中的波特率。")
    parser.add_argument("--digits", type=int, default=3, help="保存角度的小数位数。")
    parser.add_argument("--yes", action="store_true", help="确认写入配置文件；不加时只打印。")
    args = parser.parse_args()

    hit_config_path, hit_config = load_hit_config(args.hit_config)
    home_config_path = resolve_path(args.home_config)
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

    home_angles = rounded_angles(current_angles, args.digits)

    print("当前 raw 位置:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} raw={current_raw[joint]}")
    print("准备保存为 home 的角度，单位 deg:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} {home_angles[joint]:8.3f}")

    if not args.yes:
        print("当前只是 dry-run。确认这个收起姿态安全后，追加 --yes 写入 home_pose.json。")
        return

    home_pose = {
        "enabled": True,
        "start_tolerance_deg": float(hit_config.get("home_pose", {}).get("start_tolerance_deg", 8.0)),
        "joint_angles_deg": home_angles,
        "note": (
            "SO101 打靶程序使用的收起/home 位姿，单位度。"
            "hit_target_action.py 只检查当前是否接近该姿态；"
            "return_home.py 负责从当前姿态低速回到该姿态。"
        ),
    }
    home_config_path.write_text(
        json.dumps(home_pose, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    hit_config["home_pose_config"] = str(home_config_path.relative_to(REPO_ROOT)).replace("\\", "/")
    legacy_home_pose = hit_config.setdefault("home_pose", {})
    legacy_home_pose["enabled"] = True
    legacy_home_pose["return_after_hit"] = True
    legacy_home_pose["allow_outside_safe_envelope"] = True
    legacy_home_pose["joint_angles_deg"] = home_angles
    legacy_home_pose.setdefault(
        "note",
        "收起/home 位姿使用关节角表示，单位度。home 是实测安全停放姿态，默认允许位于 95% 工作安全包络外；打靶轨迹本身仍会检查安全包络。",
    )

    hit_config_path.write_text(
        json.dumps(hit_config, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"已写入 home 位姿: {home_config_path}")
    print(f"已更新兼容配置: {hit_config_path}")


if __name__ == "__main__":
    main()
