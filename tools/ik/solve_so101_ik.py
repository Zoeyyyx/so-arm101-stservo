"""SO101 末端位置 IK 求解工具。

输入当前关节角和末端位移偏移量，输出一组可交给 LeRobot `send_action`
的关节目标。这个脚本用于验证“URDF + IK + LeRobot 控制”的核心链路，
后续迁移到 ROS 时可以把这里的求解逻辑封装成 IK 节点。
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from ikpy.chain import Chain


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPO_ROOT / "assets" / "urdf" / "so101_new_calib.urdf"

# SO101 的 URDF 到 gripper_frame_link 只需要前 5 个姿态关节。
# gripper 是夹爪开合，不参与末端位姿 IK。
IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def deg_to_chain_vector(joint_degrees):
    # IKPy 的向量长度包含 base 和末端 fixed link：
    # [base, shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, fixed_tip]
    values = np.zeros(len(IK_JOINTS) + 2, dtype=float)
    for index, joint in enumerate(IK_JOINTS, start=1):
        values[index] = np.deg2rad(joint_degrees.get(joint, 0.0))
    return values


def read_joint_limits_degrees(urdf_path):
    # 从 URDF 读取关节上下限，避免 IK 求出明显超出机械结构的角度。
    root = ET.parse(urdf_path).getroot()
    limits = {}
    for joint_element in root.findall("joint"):
        name = joint_element.attrib.get("name")
        if name not in IK_JOINTS:
            continue
        limit_element = joint_element.find("limit")
        if limit_element is None:
            continue
        lower = float(limit_element.attrib["lower"])
        upper = float(limit_element.attrib["upper"])
        limits[name] = (float(np.rad2deg(lower)), float(np.rad2deg(upper)))
    return limits


def clamp_initial_degrees(joint_degrees, joint_limits, margin_deg):
    # IKPy 要求初始猜测在关节上下限内；实物读数偶尔会贴边或略超限。
    clamped = dict(joint_degrees)
    changes = []
    for joint, (lower, upper) in joint_limits.items():
        low = lower + margin_deg
        high = upper - margin_deg
        value = clamped.get(joint, 0.0)
        safe_value = min(high, max(low, value))
        clamped[joint] = safe_value
        if abs(safe_value - value) > 1e-9:
            changes.append((joint, value, safe_value, lower, upper))
    return clamped, changes


def chain_vector_to_deg(chain_values):
    return {
        joint: float(np.rad2deg(chain_values[index]))
        for index, joint in enumerate(IK_JOINTS, start=1)
    }


def parse_joint_degrees(items):
    # 命令行中使用 name=value 形式，方便直接粘贴 LeRobot observation。
    values = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"初始关节角必须写成 name=value，例如 shoulder_pan=0。收到: {item}")
        name, value = item.split("=", 1)
        name = name.strip()
        if name not in IK_JOINTS:
            raise ValueError(f"未知 IK 关节: {name}。可用关节: {', '.join(IK_JOINTS)}")
        values[name] = float(value)
    return values


def make_chain(urdf_path, use_wrist_roll):
    # base 和末端 fixed link 不参与优化。
    # 默认也冻结 wrist_roll，因为 ID5 在你的实物上有机械限位，位置 IK 不应该随便让它大幅转动。
    active_links_mask = [False, True, True, True, True, bool(use_wrist_roll), False]
    return Chain.from_urdf_file(
        str(urdf_path),
        active_links_mask=active_links_mask,
        symbolic=False,
    )


def main():
    parser = argparse.ArgumentParser(
        description="SO101 URDF + IK：给当前末端位置一个偏移，求对应关节角。"
    )
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--initial", nargs="*", default=[], help="初始关节角，单位度，例如 shoulder_lift=10")
    parser.add_argument("--dx", type=float, default=0.01, help="末端 x 方向偏移，单位米。默认 0.01m")
    parser.add_argument("--dy", type=float, default=0.0, help="末端 y 方向偏移，单位米。")
    parser.add_argument("--dz", type=float, default=0.0, help="末端 z 方向偏移，单位米。")
    parser.add_argument("--max-iter", type=int, default=200)
    parser.add_argument("--gripper", type=float, default=0.0, help="输出 send_action 命令时附带的夹爪目标。")
    parser.add_argument(
        "--limit-margin-deg",
        type=float,
        default=0.5,
        help="IK 初始值距离 URDF 上下限保留的角度余量，默认 0.5 度。",
    )
    parser.add_argument(
        "--no-clamp-initial",
        action="store_true",
        help="不自动夹紧超出 URDF limit 的初始角度。一般不建议使用。",
    )
    parser.add_argument(
        "--use-wrist-roll",
        action="store_true",
        help="允许 IK 优化 wrist_roll。默认关闭，避免 ID5 乱转。",
    )
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        raise FileNotFoundError(f"找不到 URDF: {urdf_path}")

    raw_initial_degrees = parse_joint_degrees(args.initial)
    joint_limits = read_joint_limits_degrees(urdf_path)
    if args.no_clamp_initial:
        initial_degrees = raw_initial_degrees
        clamped_changes = []
    else:
        initial_degrees, clamped_changes = clamp_initial_degrees(
            raw_initial_degrees,
            joint_limits,
            args.limit_margin_deg,
        )
    initial_chain = deg_to_chain_vector(initial_degrees)
    chain = make_chain(urdf_path, args.use_wrist_roll)

    current_fk = chain.forward_kinematics(initial_chain)
    current_xyz = current_fk[:3, 3]
    target_xyz = current_xyz + np.array([args.dx, args.dy, args.dz], dtype=float)

    solution_chain = chain.inverse_kinematics(
        target_position=target_xyz,
        initial_position=initial_chain,
        max_iter=args.max_iter,
    )
    achieved_fk = chain.forward_kinematics(solution_chain)
    achieved_xyz = achieved_fk[:3, 3]
    position_error_mm = float(np.linalg.norm(achieved_xyz - target_xyz) * 1000)

    solution_degrees = chain_vector_to_deg(solution_chain)
    send_action_targets = {**solution_degrees, "gripper": float(args.gripper)}

    print("URDF:", urdf_path)
    print("URDF 关节限位（度）:")
    for joint in IK_JOINTS:
        if joint in joint_limits:
            lower, upper = joint_limits[joint]
            print(f"  {joint}: {lower:.3f} .. {upper:.3f}")
    if clamped_changes:
        print("初始角度已被夹紧到 URDF 限位内:")
        for joint, old, new, lower, upper in clamped_changes:
            print(f"  {joint}: {old:.3f} -> {new:.3f}  (limit {lower:.3f} .. {upper:.3f})")
    active_joints = IK_JOINTS if args.use_wrist_roll else IK_JOINTS[:-1]
    print("IK 参与优化的关节:", ", ".join(active_joints))
    print("是否优化 wrist_roll:", bool(args.use_wrist_roll))
    print("当前末端 xyz（米）:", np.round(current_xyz, 5).tolist())
    print("目标末端 xyz（米）:", np.round(target_xyz, 5).tolist())
    print("求解末端 xyz（米）:", np.round(achieved_xyz, 5).tolist())
    print("位置误差（毫米）: %.3f" % position_error_mm)
    print("关节目标角度:")
    print(json.dumps(send_action_targets, indent=2, ensure_ascii=False))

    set_args = " ".join(f"{name}={value:.3f}" for name, value in send_action_targets.items())
    print("下一步可在 lerobot 环境中 dry-run 的命令:")
    print(
        "python .\\tools\\lerobot\\send_joint_action.py "
        "--port COM5 --id soarm101_follower --max-relative 5 "
        f"--set {set_args}"
    )


if __name__ == "__main__":
    main()
