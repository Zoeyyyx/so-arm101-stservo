"""一键完成“读取当前姿态 -> IK 求解 -> LeRobot 发送动作”。

用途：调试末端位置控制时，不再手动复制 observation 到 IK 命令，也不用
再复制 IK 输出到 send_action 命令。

默认只 dry-run，不会让机械臂运动；确认目标安全后显式加 `--yes`。
"""

import argparse
import os
from pathlib import Path
import sys
import time

# Windows 上 conda-forge/torch 可能同时带入两套 OpenMP runtime。
# 先设置环境变量并导入 torch，再导入 LeRobot，避免 DLL 加载顺序问题。
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import torch  # noqa: F401

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("缺少 numpy。请在 lerobot 环境中安装 numpy。") from exc

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SOFollower
from serial.tools import list_ports


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPO_ROOT / "assets" / "urdf" / "so101_new_calib.urdf"

IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
ACTION_JOINTS = IK_JOINTS + ["gripper"]


def import_ik_helpers():
    """复用 tools/ik 里的函数，避免 IK 公式在两个文件里各写一份。"""
    ik_dir = REPO_ROOT / "tools" / "ik"
    if str(ik_dir) not in sys.path:
        sys.path.insert(0, str(ik_dir))

    from solve_so101_ik import (  # noqa: PLC0415
        chain_vector_to_deg,
        clamp_initial_degrees,
        deg_to_chain_vector,
        read_joint_limits_degrees,
    )

    return chain_vector_to_deg, clamp_initial_degrees, deg_to_chain_vector, read_joint_limits_degrees


def make_chain(urdf_path, use_wrist_roll):
    try:
        from ikpy.chain import Chain
    except ImportError as exc:
        raise SystemExit(
            "缺少 ikpy。请先执行：\n"
            "conda activate lerobot\n"
            "python -m pip install ikpy"
        ) from exc

    # base 和末端 fixed link 不参与优化。
    # 默认冻结 wrist_roll，因为当前实物 ID5 有机械限位。
    active_links_mask = [False, True, True, True, True, bool(use_wrist_roll), False]
    return Chain.from_urdf_file(
        str(urdf_path),
        active_links_mask=active_links_mask,
        symbolic=False,
    )


def observation_to_joint_degrees(observation):
    """从 LeRobot observation 中提取 IK 需要的当前关节角。"""
    values = {}
    for joint in IK_JOINTS:
        key = f"{joint}.pos"
        if key not in observation:
            raise KeyError(f"observation 中缺少 {key}")
        values[joint] = float(observation[key])
    return values


def print_joint_table(title, values):
    print(title)
    for joint in ACTION_JOINTS:
        if joint in values:
            print(f"  {joint}: {float(values[joint]):.3f}")


def print_joint_delta_table(title, target_values, current_values):
    """打印目标角度相对当前角度的变化量，用来判断不同 IK 命令是否真的不同。"""
    print(title)
    for joint in ACTION_JOINTS:
        if joint not in target_values:
            continue
        current_key = f"{joint}.pos"
        if current_key not in current_values:
            continue
        delta = float(target_values[joint]) - float(current_values[current_key])
        print(f"  {joint}: {delta:+.3f}")


def format_available_ports():
    """列出当前 Windows 能看到的串口，方便排查 COM 号变化。"""
    ports = list(list_ports.comports())
    if not ports:
        return "当前没有发现任何串口。"
    lines = ["当前可见串口:"]
    for port in ports:
        lines.append(f"  {port.device}: {port.description}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="读取当前 SO101 姿态，做末端位置 IK 偏移，并通过 LeRobot 发送目标。"
    )
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--id", default="soarm101_follower")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--dx", type=float, default=0.005, help="末端 x 方向偏移，单位米。")
    parser.add_argument("--dy", type=float, default=0.0, help="末端 y 方向偏移，单位米。")
    parser.add_argument("--dz", type=float, default=0.0, help="末端 z 方向偏移，单位米。")
    parser.add_argument("--gripper", type=float, help="可选：同时设置夹爪目标；不填则保持当前夹爪值。")
    parser.add_argument("--max-iter", type=int, default=200)
    parser.add_argument(
        "--max-relative",
        type=float,
        default=25.0,
        help="LeRobot 单次动作最大相对变化量，默认 25，用于限制单次关节跳变。",
    )
    parser.add_argument(
        "--limit-margin-deg",
        type=float,
        default=0.5,
        help="IK 初始值距离 URDF 上下限保留的角度余量，默认 0.5 度。",
    )
    parser.add_argument(
        "--use-wrist-roll",
        action="store_true",
        help="允许 IK 优化 wrist_roll。默认关闭，避免 ID5 乱转。",
    )
    parser.add_argument("--hold", type=float, default=0.8)
    parser.add_argument("--yes", action="store_true", help="真正发送动作；不加时只预览。")
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        raise FileNotFoundError(f"找不到 URDF: {urdf_path}")

    (
        chain_vector_to_deg,
        clamp_initial_degrees,
        deg_to_chain_vector,
        read_joint_limits_degrees,
    ) = import_ik_helpers()

    config = SO101FollowerConfig(
        port=args.port,
        id=args.id,
        max_relative_target=args.max_relative,
    )
    robot = SOFollower(config)

    try:
        try:
            robot.connect()
        except ConnectionError as exc:
            raise SystemExit(
                f"无法连接 {args.port}。\n"
                f"{format_available_ports()}\n"
                "请确认微雪驱动板 USB 已连接、设备管理器中出现 CH343/USB-SERIAL 串口，"
                "然后用 --port 改成实际 COM 号。"
            ) from exc

        observation = robot.get_observation()
        current_degrees = observation_to_joint_degrees(observation)
        current_gripper = float(observation.get("gripper.pos", 0.0))

        joint_limits = read_joint_limits_degrees(urdf_path)
        safe_initial, clamped_changes = clamp_initial_degrees(
            current_degrees,
            joint_limits,
            args.limit_margin_deg,
        )
        initial_chain = deg_to_chain_vector(safe_initial)
        chain = make_chain(urdf_path, args.use_wrist_roll)

        current_fk = chain.forward_kinematics(initial_chain)
        current_xyz = current_fk[:3, 3]
        target_xyz = current_xyz + np.array([args.dx, args.dy, args.dz], dtype=float)

        solution_chain = chain.inverse_kinematics(
            target_position=target_xyz,
            initial_position=initial_chain,
            max_iter=args.max_iter,
        )
        achieved_xyz = chain.forward_kinematics(solution_chain)[:3, 3]
        position_error_mm = float(np.linalg.norm(achieved_xyz - target_xyz) * 1000)

        target_degrees = chain_vector_to_deg(solution_chain)
        if not args.use_wrist_roll:
            # 冻结 ID5 时，目标值保持当前 observation，避免 send_action 里出现额外变化。
            target_degrees["wrist_roll"] = current_degrees["wrist_roll"]
        target_degrees["gripper"] = current_gripper if args.gripper is None else float(args.gripper)

        action = {f"{joint}.pos": float(value) for joint, value in target_degrees.items()}

        print("当前关节角:")
        for joint in ACTION_JOINTS:
            key = f"{joint}.pos"
            if key in observation:
                print(f"  {key}: {float(observation[key]):.3f}")

        if clamped_changes:
            print("IK 初始角度已夹紧到 URDF 限位内:")
            for joint, old, new, lower, upper in clamped_changes:
                print(f"  {joint}: {old:.3f} -> {new:.3f}  (limit {lower:.3f} .. {upper:.3f})")

        print("末端位置偏移:")
        print(f"  dx={args.dx:.4f} m, dy={args.dy:.4f} m, dz={args.dz:.4f} m")
        print("IK 结果:")
        print(f"  当前末端 xyz: {np.round(current_xyz, 5).tolist()}")
        print(f"  目标末端 xyz: {np.round(target_xyz, 5).tolist()}")
        print(f"  求解末端 xyz: {np.round(achieved_xyz, 5).tolist()}")
        print(f"  位置误差: {position_error_mm:.3f} mm")
        print(f"  是否优化 wrist_roll: {bool(args.use_wrist_roll)}")
        print_joint_table("将发送的关节目标:", target_degrees)
        print_joint_delta_table("目标相对当前关节变化:", target_degrees, observation)

        if not args.yes:
            print("当前只是预览。确认机械空间安全后追加 --yes 才会调用 robot.send_action()。")
            return

        sent = robot.send_action(action)
        print("LeRobot 安全裁剪后的实际发送值:")
        for key, value in sent.items():
            print(f"  {key}: {float(value):.3f}")
        sent_targets = {key.removesuffix(".pos"): float(value) for key, value in sent.items()}
        print_joint_delta_table("安全裁剪后相对当前关节变化:", sent_targets, observation)
        time.sleep(args.hold)
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
