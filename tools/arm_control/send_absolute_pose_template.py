"""SO101 末端绝对坐标控制模板。

输入目标末端绝对位姿：
    x, y, z, roll, pitch, yaw

模板流程：
1. 将输入坐标系的目标位姿转换到机械臂基座坐标系；
2. 调用 IK 求解关节角；
3. 根据配置把关节角映射为 STS3215 原始位置；
4. 通过微雪 STservo SDK 分步平滑发送到总线舵机。

注意：
- 默认 dry-run，不会运动；必须加 `--yes` 才会发送舵机指令。
- 当前姿态 roll/pitch/yaw 已进入接口和日志，但 IK 默认只强制末端位置。
  后续接 ROS/MoveIt/更完整 IK 时，可以替换 solve_ik_pose()。
"""

import argparse
from dataclasses import dataclass
import itertools
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("缺少 numpy。请在当前环境中安装 numpy。") from exc

try:
    from ikpy.chain import Chain
except ImportError as exc:
    raise SystemExit(
        "缺少 ikpy。请先安装：\n"
        "python -m pip install ikpy"
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "config" / "absolute_pose_control.json"

# 复用项目里的微雪 STServo 共用代码，不修改官方底层通信协议。
STSERVO_TOOLS = REPO_ROOT / "tools" / "stservo"
if str(STSERVO_TOOLS) not in sys.path:
    sys.path.insert(0, str(STSERVO_TOOLS))

from stservo_common import (  # noqa: E402
    COMM_SUCCESS,
    POSITION_MAX,
    POSITION_MIN,
    check_comm,
    clamp,
    open_bus,
)


IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
ALL_JOINTS = IK_JOINTS + ["gripper"]


@dataclass
class Pose6D:
    """六维末端位姿，位置单位米，姿态单位度。"""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    frame: str


def load_config(path):
    """读取 JSON 配置，并做最基本的结构整理。"""
    config_path = Path(path)
    if not config_path.is_absolute():
        config_path = REPO_ROOT / config_path
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["_path"] = str(config_path)
    config["_joint_by_name"] = {joint["name"]: joint for joint in config["joints"]}
    return config


def get_urdf_path(config):
    """返回配置中的 URDF 绝对路径。"""
    urdf_path = Path(config["urdf"])
    if not urdf_path.is_absolute():
        urdf_path = REPO_ROOT / urdf_path
    return urdf_path


def joint_hard_limits(config):
    """读取配置中的标定关节范围。

    这里的 angle_min_deg/angle_max_deg 应该来自机械臂标定或实测极限。
    """
    return {
        joint["name"]: (float(joint["angle_min_deg"]), float(joint["angle_max_deg"]))
        for joint in config["joints"]
    }


def shrink_limit_pair(low, high, ratio):
    """把一个关节范围按中心收缩，例如 ratio=0.95 表示两侧各留 2.5%。"""
    center = (float(low) + float(high)) / 2.0
    half_span = abs(float(high) - float(low)) * float(ratio) / 2.0
    return center - half_span, center + half_span


def joint_safe_limits(config):
    """由标定关节范围派生安全关节包络。

    例如 95% 的意思是：
    - 每个关节仍然可以使用大部分标定范围；
    - 在 min/max 两端各留出 2.5% 的安全余量；
    - 这不是“每次最多转 95%”。
    """
    ratio = float(config.get("workspace", {}).get("joint_safe_ratio", 0.95))
    if not 0.0 < ratio <= 1.0:
        raise ValueError("workspace.joint_safe_ratio 必须在 0..1 之间")
    return {
        name: shrink_limit_pair(low, high, ratio)
        for name, (low, high) in joint_hard_limits(config).items()
    }


def print_joint_envelope(config, safe_limits):
    """打印标定关节范围和派生出的 95% 安全范围。"""
    ratio = float(config.get("workspace", {}).get("joint_safe_ratio", 0.95))
    print(f"关节安全包络：使用标定范围的 {ratio * 100:.1f}%")
    for joint in ALL_JOINTS:
        hard_min, hard_max = joint_hard_limits(config)[joint]
        safe_min, safe_max = safe_limits[joint]
        print(
            f"  {joint:14s} hard={hard_min:8.3f}..{hard_max:8.3f} deg  "
            f"safe={safe_min:8.3f}..{safe_max:8.3f} deg"
        )


def transform_target_pose_to_base(target_pose, config):
    """坐标系转换占位函数。

    当前模板默认输入坐标已经是机械臂基座坐标。
    后续接视觉/ROS 时，在这里调用 TF、相机外参或手写矩阵，把：
        camera/world/map 坐标
    转换成：
        so101_base 坐标
    """
    base_frame = config["frames"]["robot_base_frame"]
    return Pose6D(
        x=target_pose.x,
        y=target_pose.y,
        z=target_pose.z,
        roll=target_pose.roll,
        pitch=target_pose.pitch,
        yaw=target_pose.yaw,
        frame=base_frame,
    )


def clamp_angle(joint_config, angle_deg):
    """把角度限制到配置的安全角度范围内。"""
    return clamp(
        float(angle_deg),
        float(joint_config["angle_min_deg"]),
        float(joint_config["angle_max_deg"]),
    )


def angle_to_raw(joint_config, angle_deg):
    """把关节角度线性映射为 STS3215 原始位置。

    如果某个关节方向相反，可以在配置里交换 raw_min/raw_max。
    """
    angle_min = float(joint_config["angle_min_deg"])
    angle_max = float(joint_config["angle_max_deg"])
    raw_min = float(joint_config["raw_min"])
    raw_max = float(joint_config["raw_max"])
    if angle_max == angle_min:
        raise ValueError(f"{joint_config['name']} 的 angle_min_deg 和 angle_max_deg 不能相同")

    angle = float(angle_deg)
    ratio = (angle - angle_min) / (angle_max - angle_min)
    raw = raw_min + ratio * (raw_max - raw_min)
    return int(round(clamp(raw, POSITION_MIN, POSITION_MAX)))


def raw_to_angle(joint_config, raw_position):
    """把 STS3215 原始位置反算为模板角度。

    这里不做夹紧：如果 raw 已经超出配置的标定范围，要让后续安全检查看见。
    """
    raw_min = float(joint_config["raw_min"])
    raw_max = float(joint_config["raw_max"])
    angle_min = float(joint_config["angle_min_deg"])
    angle_max = float(joint_config["angle_max_deg"])
    if raw_max == raw_min:
        raise ValueError(f"{joint_config['name']} 的 raw_min 和 raw_max 不能相同")

    ratio = (float(raw_position) - raw_min) / (raw_max - raw_min)
    return angle_min + ratio * (angle_max - angle_min)


def read_current_joint_angles(packet_handler, config):
    """从总线读取当前六个舵机位置，并按配置转换成角度。"""
    current_angles = {}
    current_raw = {}
    for joint_config in config["joints"]:
        scs_id = int(joint_config["id"])
        raw_position, speed, result, error = packet_handler.ReadPosSpeed(scs_id)
        check_comm(packet_handler, scs_id, result, error, "ReadPosSpeed")
        current_raw[joint_config["name"]] = int(raw_position)
        current_angles[joint_config["name"]] = raw_to_angle(joint_config, raw_position)
    return current_angles, current_raw


def joint_degrees_to_chain_vector(joint_degrees):
    """把关节角字典转换成 IKPy 需要的向量。"""
    values = np.zeros(len(IK_JOINTS) + 2, dtype=float)
    for index, joint in enumerate(IK_JOINTS, start=1):
        values[index] = np.deg2rad(float(joint_degrees.get(joint, 0.0)))
    return values


def chain_vector_to_joint_degrees(chain_values):
    """把 IKPy 求解结果转换成关节角字典。"""
    return {
        joint: float(np.rad2deg(chain_values[index]))
        for index, joint in enumerate(IK_JOINTS, start=1)
    }


def make_chain(urdf_path, use_wrist_roll, safe_limits=None):
    """创建 IK 链。

    默认冻结 wrist_roll，避免 ID5 在存在物理限位时被 IK 大幅旋转。
    """
    active_links_mask = [False, True, True, True, True, bool(use_wrist_roll), False]
    chain = Chain.from_urdf_file(
        str(urdf_path),
        active_links_mask=active_links_mask,
        symbolic=False,
    )
    if safe_limits:
        for index, joint in enumerate(IK_JOINTS, start=1):
            safe_min, safe_max = safe_limits[joint]
            chain.links[index].bounds = (float(np.deg2rad(safe_min)), float(np.deg2rad(safe_max)))
    return chain


def read_urdf_joint_limits_degrees(urdf_path):
    """读取 URDF 中 IK 关节的角度限位。"""
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


def clamp_initial_angles_to_limits(current_angles, limits, margin_deg):
    """把 IK 初始角夹到指定 limit 内，避免 IKPy 报 bounds 错误。"""
    safe_angles = dict(current_angles)
    changes = []
    for joint, (lower, upper) in limits.items():
        low = lower + margin_deg
        high = upper - margin_deg
        old = float(safe_angles.get(joint, 0.0))
        new = clamp(old, low, high)
        safe_angles[joint] = new
        if abs(new - old) > 1e-9:
            changes.append(
                {
                    "joint": joint,
                    "old": old,
                    "new": new,
                    "limit_min": lower,
                    "limit_max": upper,
                }
            )
    return safe_angles, changes


def chain_vector_from_partial_degrees(joint_degrees, defaults=None):
    """用部分关节角生成 IKPy 向量，缺省关节使用 defaults 或 0。"""
    values = dict(defaults or {})
    values.update(joint_degrees)
    return joint_degrees_to_chain_vector(values)


def compute_workspace_bounds(config, safe_limits):
    """用安全关节包络采样 FK，得到末端可达工作空间的 AABB 估计。

    这是由真实关节标定范围派生出来的工作空间，不是手写经验半径。
    AABB 只负责快速判断目标是否明显在包络外；即使在 AABB 内，仍需 IK
    和关节极限检查确认是否真实可达。
    """
    urdf_path = get_urdf_path(config)
    use_wrist_roll = bool(config["ik"].get("use_wrist_roll", False))
    chain = make_chain(urdf_path, use_wrist_roll, safe_limits=safe_limits)
    sample_count = max(2, int(config.get("workspace", {}).get("sample_count_per_joint", 5)))

    sampled_joints = IK_JOINTS if use_wrist_roll else IK_JOINTS[:-1]
    axes = [
        np.linspace(safe_limits[joint][0], safe_limits[joint][1], sample_count)
        for joint in sampled_joints
    ]
    defaults = {
        joint: (safe_limits[joint][0] + safe_limits[joint][1]) / 2.0
        for joint in IK_JOINTS
    }

    points = []
    for combo in itertools.product(*axes):
        joint_values = dict(zip(sampled_joints, combo))
        chain_vector = chain_vector_from_partial_degrees(joint_values, defaults=defaults)
        xyz = chain.forward_kinematics(chain_vector)[:3, 3]
        points.append(xyz)

    cloud = np.array(points, dtype=float)
    return {
        "min": cloud.min(axis=0),
        "max": cloud.max(axis=0),
        "samples": len(points),
        "sampled_joints": sampled_joints,
    }


def print_workspace_bounds(workspace_bounds):
    """打印由安全关节包络采样得到的末端工作空间范围。"""
    xyz_min = workspace_bounds["min"]
    xyz_max = workspace_bounds["max"]
    print("安全工作空间 AABB 估计（由 95% 关节包络 FK 采样得到）:")
    print(f"  sampled_joints={', '.join(workspace_bounds['sampled_joints'])}")
    print(f"  samples={workspace_bounds['samples']}")
    print(f"  x: {xyz_min[0]: .4f} .. {xyz_max[0]: .4f} m")
    print(f"  y: {xyz_min[1]: .4f} .. {xyz_max[1]: .4f} m")
    print(f"  z: {xyz_min[2]: .4f} .. {xyz_max[2]: .4f} m")


def solve_ik_pose(target_pose_base, current_angles, config, safe_limits, gripper_target=None):
    """根据目标绝对位姿求解六个舵机目标角度。

    当前模板使用 IKPy 做位置 IK：
    - x/y/z 会作为末端绝对位置参与求解；
    - roll/pitch/yaw 先进入函数接口和日志，暂不强制约束；
    - gripper 不参与 IK，默认保持当前值。

    后续要做真正 6D 位姿控制时，可以在这里替换为 MoveIt、KDL、
    Pinocchio、placo 或自写 IK。
    """
    urdf_path = get_urdf_path(config)
    urdf_limits = read_urdf_joint_limits_degrees(urdf_path)
    ik_limits = {
        joint: (
            max(safe_limits[joint][0], urdf_limits[joint][0]),
            min(safe_limits[joint][1], urdf_limits[joint][1]),
        )
        for joint in IK_JOINTS
    }
    safe_initial_angles, clamped_initial = clamp_initial_angles_to_limits(
        current_angles,
        ik_limits,
        float(config["ik"].get("limit_margin_deg", 0.5)),
    )

    chain = make_chain(
        urdf_path,
        bool(config["ik"].get("use_wrist_roll", False)),
        safe_limits=ik_limits,
    )
    initial_chain = joint_degrees_to_chain_vector(safe_initial_angles)
    target_position = np.array([target_pose_base.x, target_pose_base.y, target_pose_base.z], dtype=float)

    solution_chain = chain.inverse_kinematics(
        target_position=target_position,
        initial_position=initial_chain,
        max_iter=int(config["ik"].get("max_iter", 200)),
    )
    achieved_position = chain.forward_kinematics(solution_chain)[:3, 3]
    position_error_mm = float(np.linalg.norm(achieved_position - target_position) * 1000.0)

    raw_target_angles = chain_vector_to_joint_degrees(solution_chain)
    target_angles = dict(raw_target_angles)
    if not bool(config["ik"].get("use_wrist_roll", False)):
        target_angles["wrist_roll"] = current_angles["wrist_roll"]
    target_angles["gripper"] = current_angles["gripper"] if gripper_target is None else float(gripper_target)

    debug = {
        "target_position": target_position.tolist(),
        "achieved_position": achieved_position.tolist(),
        "position_error_mm": position_error_mm,
        "orientation_note": "roll/pitch/yaw 已接收，但当前模板默认不强制姿态 IK。",
        "clamped_initial": clamped_initial,
        "raw_ik_target_angles": raw_target_angles,
        "ik_limits": ik_limits,
    }
    return target_angles, debug


def check_workspace_safety(target_pose_base, workspace_bounds, config):
    """检查目标是否落在安全关节包络采样得到的末端 AABB 内。"""
    margin = float(config.get("workspace", {}).get("aabb_margin_m", 0.0))
    xyz = np.array([target_pose_base.x, target_pose_base.y, target_pose_base.z], dtype=float)
    xyz_min = np.array(workspace_bounds["min"], dtype=float) - margin
    xyz_max = np.array(workspace_bounds["max"], dtype=float) + margin
    violations = []
    for axis, value, low, high in zip(["x", "y", "z"], xyz, xyz_min, xyz_max):
        if value < low or value > high:
            violations.append(f"目标 {axis}={value:.4f} m 不在工作空间范围 {low:.4f}..{high:.4f} m 内")
    return violations


def check_current_joint_envelope(current_angles, safe_limits):
    """检查当前关节是否已经位于安全包络外。"""
    violations = []
    for joint in ALL_JOINTS:
        low, high = safe_limits[joint]
        value = float(current_angles[joint])
        if value < low or value > high:
            violations.append((joint, value, low, high))
    return violations


def outside_safe_envelope_joints(current_angles, safe_limits):
    """返回当前已经在安全包络外的关节名集合。"""
    return {joint for joint, value, low, high in check_current_joint_envelope(current_angles, safe_limits)}


def check_target_joint_envelope(target_angles, safe_limits):
    """检查 IK 目标是否落在 95% 安全关节包络内。"""
    violations = []
    for joint in ALL_JOINTS:
        low, high = safe_limits[joint]
        value = float(target_angles[joint])
        if value < low or value > high:
            violations.append((joint, value, low, high))
    return violations


def interpolate_joint_angles(current_angles, target_angles, steps):
    """按关节角线性插值，生成平滑运动路径。"""
    steps = max(1, int(steps))
    path = []
    for step in range(1, steps + 1):
        ratio = step / steps
        point = {}
        for joint in ALL_JOINTS:
            start = float(current_angles[joint])
            end = float(target_angles[joint])
            point[joint] = start + (end - start) * ratio
        path.append(point)
    return path


def choose_motion_profile(config, recovery_mode):
    """根据是否处于上电恢复模式选择速度/加速度。"""
    motion = config["motion"]
    if recovery_mode:
        return (
            int(motion.get("startup_recovery_speed", motion["speed"])),
            int(motion.get("startup_recovery_acc", motion["acc"])),
        )
    return int(motion["speed"]), int(motion["acc"])


def send_joint_point(packet_handler, joint_point, config, speed, acc):
    """把一帧关节角目标转换为 raw 位置，并发送给六个 STS3215 舵机。"""
    raw_targets = {}
    for joint_config in config["joints"]:
        name = joint_config["name"]
        scs_id = int(joint_config["id"])
        raw = angle_to_raw(joint_config, joint_point[name])
        result, error = packet_handler.WritePosEx(scs_id, raw, int(speed), int(acc))
        check_comm(packet_handler, scs_id, result, error, "WritePosEx")
        raw_targets[name] = raw
    return raw_targets


def print_joint_report(current_angles, target_angles, target_raw):
    """打印当前角、目标角、角度变化和 raw 目标。"""
    print("关节目标报告:")
    for joint in ALL_JOINTS:
        delta = float(target_angles[joint]) - float(current_angles[joint])
        print(
            f"  {joint:14s} current={current_angles[joint]:8.3f} "
            f"target={target_angles[joint]:8.3f} "
            f"delta={delta:+8.3f} raw={target_raw[joint]}"
        )


def main():
    parser = argparse.ArgumentParser(description="SO101 末端绝对坐标控制模板。")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--port", help="覆盖配置文件中的 COM 端口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖配置文件中的波特率。")
    parser.add_argument("--frame", default="world", help="输入目标所在坐标系名。")
    parser.add_argument("--x", type=float, help="目标 x，单位米。")
    parser.add_argument("--y", type=float, help="目标 y，单位米。")
    parser.add_argument("--z", type=float, help="目标 z，单位米。")
    parser.add_argument("--roll", type=float, default=0.0, help="目标 roll，单位度。")
    parser.add_argument("--pitch", type=float, default=0.0, help="目标 pitch，单位度。")
    parser.add_argument("--yaw", type=float, default=0.0, help="目标 yaw，单位度。")
    parser.add_argument("--gripper", type=float, help="夹爪目标；不填则保持当前值。")
    parser.add_argument("--steps", type=int, help="覆盖平滑插值步数。")
    parser.add_argument("--dt", type=float, help="覆盖每步延时，单位秒。")
    parser.add_argument("--speed", type=int, help="覆盖 STS3215 速度参数。")
    parser.add_argument("--acc", type=int, help="覆盖 STS3215 加速度参数。")
    parser.add_argument("--workspace-samples", type=int, help="覆盖每个关节的工作空间采样数量。")
    parser.add_argument("--show-workspace", action="store_true", help="只打印 95%% 安全关节包络和末端工作空间范围，不连接舵机。")
    parser.add_argument("--yes", action="store_true", help="真正发送舵机指令；不加时只预览。")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.port:
        config["serial"]["port"] = args.port
    if args.baudrate:
        config["serial"]["baudrate"] = args.baudrate
    if args.steps is not None:
        config["motion"]["steps"] = args.steps
    if args.dt is not None:
        config["motion"]["dt"] = args.dt
    if args.speed is not None:
        config["motion"]["speed"] = args.speed
        config["motion"]["startup_recovery_speed"] = args.speed
    if args.acc is not None:
        config["motion"]["acc"] = args.acc
        config["motion"]["startup_recovery_acc"] = args.acc
    if args.workspace_samples is not None:
        config["workspace"]["sample_count_per_joint"] = args.workspace_samples

    safe_limits = joint_safe_limits(config)
    workspace_bounds = compute_workspace_bounds(config, safe_limits)
    print_joint_envelope(config, safe_limits)
    print_workspace_bounds(workspace_bounds)

    if args.show_workspace:
        return

    if args.x is None or args.y is None or args.z is None:
        raise SystemExit("缺少目标位置。请提供 --x --y --z，或使用 --show-workspace 只查看工作空间。")

    target_pose = Pose6D(args.x, args.y, args.z, args.roll, args.pitch, args.yaw, args.frame)
    target_pose_base = transform_target_pose_to_base(target_pose, config)

    workspace_violations = check_workspace_safety(target_pose_base, workspace_bounds, config)
    if workspace_violations:
        print("工作空间判断：目标不在由 95% 关节包络得到的安全末端范围内。")
        for item in workspace_violations:
            print(f"  {item}")
        print("已拒绝进入 IK 和舵机执行。请换目标点，或重新核对 calibration/安全包络配置。")
        return

    port_handler, packet_handler = open_bus(
        config["serial"]["port"],
        int(config["serial"]["baudrate"]),
    )

    try:
        current_angles, current_raw = read_current_joint_angles(packet_handler, config)
        target_angles, ik_debug = solve_ik_pose(
            target_pose_base,
            current_angles,
            config,
            safe_limits,
            gripper_target=args.gripper,
        )
        target_raw = {
            name: angle_to_raw(config["_joint_by_name"][name], angle)
            for name, angle in target_angles.items()
        }

        print("配置文件:", config["_path"])
        print(f"串口: {config['serial']['port']}  波特率: {config['serial']['baudrate']}")
        print("输入目标位姿:")
        print(
            f"  frame={target_pose.frame} "
            f"x={target_pose.x:.4f} y={target_pose.y:.4f} z={target_pose.z:.4f} "
            f"roll={target_pose.roll:.2f} pitch={target_pose.pitch:.2f} yaw={target_pose.yaw:.2f}"
        )
        print("转换到机械臂基座坐标后的目标:")
        print(
            f"  frame={target_pose_base.frame} "
            f"x={target_pose_base.x:.4f} y={target_pose_base.y:.4f} z={target_pose_base.z:.4f} "
            f"roll={target_pose_base.roll:.2f} pitch={target_pose_base.pitch:.2f} yaw={target_pose_base.yaw:.2f}"
        )
        print("IK 调试信息:")
        print(f"  target_position={np.round(ik_debug['target_position'], 5).tolist()}")
        print(f"  achieved_position={np.round(ik_debug['achieved_position'], 5).tolist()}")
        print(f"  position_error_mm={ik_debug['position_error_mm']:.3f}")
        print(f"  {ik_debug['orientation_note']}")
        if ik_debug["clamped_initial"]:
            print("IK 初始角已夹紧到 95% 安全关节包络内:")
            for item in ik_debug["clamped_initial"]:
                print(
                    f"  {item['joint']}: {item['old']:.3f} -> {item['new']:.3f} "
                    f"(safe {item['limit_min']:.3f} .. {item['limit_max']:.3f})"
                )
        print("当前 raw 位置:")
        for joint in ALL_JOINTS:
            print(f"  {joint:14s} raw={current_raw[joint]}")
        print_joint_report(current_angles, target_angles, target_raw)

        current_envelope_violations = check_current_joint_envelope(current_angles, safe_limits)
        startup_recovery_mode = bool(current_envelope_violations)
        if current_envelope_violations:
            print("当前关节不在 95% 安全包络内：")
            for joint, value, low, high in current_envelope_violations:
                print(f"  {joint}: current={value:.3f} deg, safe={low:.3f}..{high:.3f} deg")
            if bool(config.get("workspace", {}).get("allow_startup_recovery_from_outside_safe_envelope", True)):
                print("进入上电恢复模式：当前姿态在安全包络外，但只要目标在安全包络内，允许向安全目标运动。")
                print("恢复模式会使用更保守的 speed/acc。")
            else:
                print("已拒绝发送。请先检查 calibration/config 映射，或手动把机械臂移动回安全包络。")
                return

        target_envelope_violations = check_target_joint_envelope(target_angles, safe_limits)
        if target_envelope_violations:
            print("IK 目标不在 95% 安全关节包络内：")
            for joint, value, low, high in target_envelope_violations:
                print(f"  {joint}: target={value:.3f} deg, safe={low:.3f}..{high:.3f} deg")
            print("已拒绝发送。该目标不可达或过于贴近标定极限，请换目标点。")
            return

        if startup_recovery_mode:
            outside_joints = outside_safe_envelope_joints(current_angles, safe_limits)
            not_recovered = []
            for joint in outside_joints:
                low, high = safe_limits[joint]
                target = float(target_angles[joint])
                if target < low or target > high:
                    not_recovered.append((joint, target, low, high))
            if not_recovered:
                print("已拒绝发送：目标没有把以下越界关节带回安全包络内。")
                for joint, target, low, high in not_recovered:
                    print(f"  {joint}: target={target:.3f} deg, safe={low:.3f}..{high:.3f} deg")
                return

        max_error_mm = float(config.get("workspace", {}).get("position_error_max_mm", 10.0))
        if float(ik_debug["position_error_mm"]) > max_error_mm:
            print(
                f"已拒绝发送：IK 位置误差 {ik_debug['position_error_mm']:.3f} mm "
                f"> 允许误差 {max_error_mm:.3f} mm。"
            )
            print("目标虽然可能落在 AABB 内，但不一定真实可达；请换目标点。")
            return

        if not args.yes:
            print("当前只是 dry-run。确认机械空间安全后追加 --yes 才会发送舵机指令。")
            return

        path = interpolate_joint_angles(current_angles, target_angles, config["motion"]["steps"])
        speed, acc = choose_motion_profile(config, startup_recovery_mode)
        print(
            f"开始发送：steps={config['motion']['steps']} "
            f"dt={config['motion']['dt']} speed={speed} acc={acc}"
        )
        last_raw_targets = {}
        for point in path:
            last_raw_targets = send_joint_point(
                packet_handler,
                point,
                config,
                speed,
                acc,
            )
            time.sleep(float(config["motion"]["dt"]))

        print("发送完成，最后一帧 raw 目标:")
        for joint in ALL_JOINTS:
            print(f"  {joint:14s} raw={last_raw_targets[joint]}")
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    main()
