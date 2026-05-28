"""人工记录 SO101 机械臂末端禁入区域边界点。

使用方法：
1. 如需手动拖动，先用 tools/stservo/torque_off.py 释放扭矩；
2. 把机械臂摆到“刚好危险 / 不希望进入”的边界位置；
3. 按 Enter 记录当前关节角、raw 位置和 URDF/FK 末端坐标；
4. 输入 q 退出。

本脚本默认只读，不持续发送保持指令，也不修改底层 SDK。
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("缺少 numpy。请在当前环境中安装 numpy。") from exc


REPO_ROOT = Path(__file__).resolve().parents[2]
ARM_CONTROL_DIR = REPO_ROOT / "tools" / "arm_control"
if str(ARM_CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(ARM_CONTROL_DIR))

from send_absolute_pose_template import (  # noqa: E402
    ALL_JOINTS,
    get_urdf_path,
    joint_degrees_to_chain_vector,
    load_config,
    make_chain,
    open_bus,
    read_current_joint_angles,
)


DEFAULT_CONFIG = REPO_ROOT / "config" / "absolute_pose_control.json"
DEFAULT_OUTPUT = REPO_ROOT / "config" / "forbidden_zone_points.json"


def resolve_path(path):
    """把相对路径解析到项目根目录下。"""
    result = Path(path)
    if not result.is_absolute():
        result = REPO_ROOT / result
    return result


def now_iso():
    """生成本地时间戳。"""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load_points_file(path, robot_id, port):
    """读取或初始化人工记录文件。"""
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {
        "timestamp": now_iso(),
        "updated_at": now_iso(),
        "robot_id": robot_id,
        "port": port,
        "frame": "so101_base",
        "note": "人工记录的危险边界点，仅用于生成简化禁入区域；不是精确碰撞模型。",
        "samples": [],
    }


def end_effector_position(chain, joint_angles):
    """根据当前关节角计算末端在 base 坐标系下的位置，单位米。"""
    chain_vector = joint_degrees_to_chain_vector(joint_angles)
    xyz = chain.forward_kinematics(chain_vector)[:3, 3]
    return [float(value) for value in xyz]


def read_sample(packet_handler, config, chain, label):
    """读取当前舵机状态并计算 FK。"""
    joint_angles, raw_positions = read_current_joint_angles(packet_handler, config)
    position = end_effector_position(chain, joint_angles)
    return {
        "index": None,
        "timestamp": now_iso(),
        "label": label,
        "joint_angles_deg": {joint: round(float(joint_angles[joint]), 6) for joint in ALL_JOINTS},
        "raw_position": {joint: int(raw_positions[joint]) for joint in ALL_JOINTS},
        "end_effector_position": {
            "x": round(position[0], 6),
            "y": round(position[1], 6),
            "z": round(position[2], 6),
        },
    }


def print_sample(sample):
    """打印一个记录点。"""
    pos = sample["end_effector_position"]
    print(
        f"已记录 #{sample['index']:03d} label={sample['label']} "
        f"x={pos['x']:.4f} y={pos['y']:.4f} z={pos['z']:.4f} m"
    )
    print("当前 joint angle deg:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} {sample['joint_angles_deg'][joint]:9.3f}")
    print("当前 raw:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} {sample['raw_position'][joint]}")


def save_points(path, data):
    """保存记录文件。"""
    data["updated_at"] = now_iso()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="人工记录机械臂末端禁入区域边界点。")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="绝对坐标控制配置。")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="记录点输出 JSON。")
    parser.add_argument("--port", help="覆盖配置中的 COM 口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖配置中的波特率。")
    parser.add_argument("--id", default="soarm101_follower", help="机器人 ID，仅写入记录文件。")
    parser.add_argument(
        "--label",
        default="base_collision_boundary",
        help="记录点默认标签，例如 gripper_near_base / return_path_collision。",
    )
    args = parser.parse_args()

    config = load_config(resolve_path(args.config))
    if args.port:
        config["serial"]["port"] = args.port
    if args.baudrate:
        config["serial"]["baudrate"] = args.baudrate

    output_path = resolve_path(args.output)
    data = load_points_file(output_path, args.id, config["serial"]["port"])
    data["robot_id"] = args.id
    data["port"] = config["serial"]["port"]

    urdf_path = get_urdf_path(config)
    chain = make_chain(urdf_path, bool(config["ik"].get("use_wrist_roll", False)))

    print("连接机械臂总线:")
    print(f"  port={config['serial']['port']} baudrate={config['serial']['baudrate']}")
    print(f"  robot_id={args.id}")
    print(f"  urdf={urdf_path}")
    print("提示：本脚本只读取当前位置，不发送保持指令。")
    print("请手动将机械臂摆到危险边界位置，然后按 Enter 记录；输入 q 退出。")

    port_handler, packet_handler = open_bus(config["serial"]["port"], int(config["serial"]["baudrate"]))
    try:
        while True:
            text = input("Enter=记录 / q=退出 / label=新标签 > ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            label = args.label if text == "" else text
            sample = read_sample(packet_handler, config, chain, label)
            sample["index"] = len(data["samples"]) + 1
            data["samples"].append(sample)
            save_points(output_path, data)
            print_sample(sample)
            print(f"已保存到: {output_path}")
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    main()
