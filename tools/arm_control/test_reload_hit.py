"""Standalone reload position and reload press diagnostic.

This script does not call the full hit planner.  It only builds:
home -> move_to_reload -> reload_hit_down -> reload_hold -> reload_hit_up -> return_home
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
ARM_CONTROL_ROOT = REPO_ROOT / "tools" / "arm_control"
STSERVO_ROOT = REPO_ROOT / "tools" / "stservo"
for path in (ARM_CONTROL_ROOT, STSERVO_ROOT):
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)

from core.config_loader import load_controller_config, load_hit_config, load_home_pose, load_json  # noqa: E402
from core.ik_solver import IKSolver, normalize_vector, orientation_error_deg  # noqa: E402
from core.safety_checker import SafetyChecker  # noqa: E402
from core.servo_interface import ServoInterface, target_raw_from_angles  # noqa: E402
from core.trajectory_planner import HitTrajectoryPlanner, phase_profile  # noqa: E402
from core.types import ALL_JOINTS, ACTIVE_JOINTS, Pose6D, TrajectoryPoint  # noqa: E402
from send_absolute_pose_template import compute_workspace_bounds, joint_degrees_to_chain_vector, joint_safe_limits  # noqa: E402


DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"
DEFAULT_RELOAD_POSE = REPO_ROOT / "config" / "reload_pose.json"
FOCUS_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def build_parser():
    parser = argparse.ArgumentParser(description="Standalone SO101 reload hit diagnostic.")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG))
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument("--reload-pose", default=str(DEFAULT_RELOAD_POSE))
    parser.add_argument("--port", default=None)
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--reload-contact-z", type=float, default=None)
    parser.add_argument("--position-error-max-mm", type=float, default=15.0)
    parser.add_argument("--orientation-error-max-deg", type=float, default=15.0)
    parser.add_argument(
        "--strict-tool-down",
        action="store_true",
        help="Disable position-first fallback and fail the contract when tool axis error is too large.",
    )
    parser.add_argument("--wait-raw-tolerance", type=int, default=45)
    parser.add_argument("--wait-timeout", type=float, default=1.2)
    parser.add_argument("--wait-poll-interval", type=float, default=0.02)
    parser.add_argument("--yes", action="store_true", help="Actually send the reload diagnostic trajectory.")
    return parser


def configured_joint_angles(pose_config, fallback=None):
    fallback = fallback or {}
    configured = pose_config.get("joint_angles_deg", {})
    missing = [joint for joint in ALL_JOINTS if joint not in configured and joint not in fallback]
    if missing:
        raise ValueError(f"Missing joint angles in pose config: {missing}")
    angles = {}
    for joint in ALL_JOINTS:
        angles[joint] = float(configured.get(joint, fallback.get(joint)))
    return angles


def fk_matrix(ik_solver, angles):
    chain_vector = joint_degrees_to_chain_vector(angles)
    return ik_solver.chain.forward_kinematics(chain_vector)


def tool_axis_from_matrix(matrix, orientation_mode):
    if orientation_mode == "X":
        return normalize_vector(matrix[:3, 0])
    if orientation_mode == "Y":
        return normalize_vector(matrix[:3, 1])
    return normalize_vector(matrix[:3, 2])


def pose_xyz(pose):
    return float(pose.x), float(pose.y), float(pose.z)


def format_xyz(values):
    return f"x={float(values[0]):.6f} y={float(values[1]):.6f} z={float(values[2]):.6f}"


def print_joint_table(title, angles, raw=None):
    print(title)
    for joint in ALL_JOINTS:
        suffix = f" raw={int(raw[joint])}" if raw and joint in raw else ""
        print(f"  {joint:14s} angle={float(angles[joint]):9.3f} deg{suffix}")


def assert_reload_hit_contract(
    reload_down_points,
    debug_rows,
    reload_above,
    reload_contact,
    *,
    max_position_error_mm,
    max_orientation_error_deg,
    strict_tool_down=False,
):
    tolerance_m = 1e-9
    ok = True
    z_values = [float(point.pose.z) for point in reload_down_points]
    x_values = [float(point.pose.x) for point in reload_down_points]
    y_values = [float(point.pose.y) for point in reload_down_points]
    xy_constant = (
        all(abs(value - float(reload_above.x)) <= tolerance_m for value in x_values)
        and all(abs(value - float(reload_above.y)) <= tolerance_m for value in y_values)
    )
    z_monotonic = all(current <= previous + tolerance_m for previous, current in zip(z_values, z_values[1:]))
    target_reaches_contact = abs(z_values[-1] - float(reload_contact.z)) <= tolerance_m if z_values else False

    print("reload_hit_down point diagnostics:")
    for row in debug_rows:
        target = row["target"]
        achieved = row["achieved"]
        actual_axis = row["actual_tool_axis"]
        print(
            f"  #{row['index']:02d} "
            f"target=({format_xyz(target)}) "
            f"achieved=({format_xyz(achieved)}) "
            f"position_error_mm={row['position_error_mm']:.3f} "
            f"orientation_error_deg={row['orientation_error_deg']:.3f} "
            f"orientation_requested={row['orientation_requested']} "
            f"orientation_fallback={row['orientation_fallback']}"
        )
        print(
            f"      actual_tool_axis=[{actual_axis[0]:+.6f}, {actual_axis[1]:+.6f}, {actual_axis[2]:+.6f}] "
            f"target_tool_axis=[0.000000, 0.000000, -1.000000]"
        )
        if row["position_error_mm"] > max_position_error_mm:
            ok = False
            print("      FAIL position error above threshold")
        if row["orientation_error_deg"] is None or row["orientation_error_deg"] > max_orientation_error_deg:
            if strict_tool_down:
                ok = False
                print("      FAIL tool-down orientation error above threshold")
            else:
                print("      WARN tool-down orientation error above threshold")

    print("reload_hit_down contract:")
    print(f"  z_range: start={z_values[0]:.6f} end={z_values[-1]:.6f}" if z_values else "  z_range: empty")
    print(f"  xy_constant={xy_constant}")
    print(f"  z_monotonic_down={z_monotonic}")
    print(f"  target_reaches_reload_contact_z={target_reaches_contact}")
    print(f"  reload_contact_target_z={float(reload_contact.z):.6f}")
    if not xy_constant or not z_monotonic or not target_reaches_contact:
        ok = False
    max_orientation_error = max(
        (
            float(row["orientation_error_deg"])
            for row in debug_rows
            if row["orientation_error_deg"] is not None
        ),
        default=float("nan"),
    )
    orientation_ok = bool(
        max_orientation_error <= float(max_orientation_error_deg)
        if not np.isnan(max_orientation_error)
        else False
    )
    return {
        "ok": bool(ok),
        "xy_constant": bool(xy_constant),
        "z_monotonic_down": bool(z_monotonic),
        "target_reaches_reload_contact_z": bool(target_reaches_contact),
        "max_position_error_mm": max((float(row["position_error_mm"]) for row in debug_rows), default=0.0),
        "max_orientation_error_deg": max_orientation_error,
        "orientation_ok": orientation_ok,
        "strict_tool_down": bool(strict_tool_down),
    }


def build_reload_diagnostic(args):
    hit_config = load_hit_config(args.hit_config)
    home_pose = load_home_pose(hit_config, args.home_config)
    reload_pose = load_json(args.reload_pose)
    controller_config = load_controller_config(hit_config)
    if args.port:
        controller_config["serial"]["port"] = args.port
    if args.baudrate:
        controller_config["serial"]["baudrate"] = int(args.baudrate)

    tool_config = hit_config["tool_orientation"]
    tool_config["enforce_tool_down"] = True
    if args.strict_tool_down:
        tool_config["position_first_fallback"] = False
    tool_config["target_axis_in_base"] = [0.0, 0.0, -1.0]
    tool_down_phases = set(tool_config.get("tool_down_phases", []))
    tool_down_phases.add("reload_hit_down")
    tool_config["tool_down_phases"] = sorted(tool_down_phases)

    safe_limits = joint_safe_limits(controller_config)
    workspace_bounds = compute_workspace_bounds(controller_config, safe_limits)
    ik_solver = IKSolver(controller_config, safe_limits)
    safety_checker = SafetyChecker(controller_config, safe_limits, workspace_bounds, hit_config=hit_config)

    home_angles = configured_joint_angles(home_pose)
    reload_angles = configured_joint_angles(reload_pose, fallback=home_angles)
    reload_raw = target_raw_from_angles(reload_angles, controller_config)
    home_raw = target_raw_from_angles(home_angles, controller_config)

    frame = str(reload_pose.get("frame", hit_config["frames"]["robot_base_frame"]))
    template = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, frame)
    reload_above = ik_solver.fk_pose(reload_angles, template)
    contact_z = float(
        args.reload_contact_z
        if args.reload_contact_z is not None
        else reload_pose.get("reload_contact_z_m", 0.010)
    )
    reload_contact = Pose6D(
        x=float(reload_above.x),
        y=float(reload_above.y),
        z=contact_z,
        roll=float(reload_above.roll),
        pitch=float(reload_above.pitch),
        yaw=float(reload_above.yaw),
        frame=reload_above.frame,
    )

    print(f"reload_pose_path={reload_pose['_path']}")
    print_joint_table("reload joint angles/raw:", reload_angles, reload_raw)
    print(f"reload_above target xyz: {format_xyz(pose_xyz(reload_above))} frame={reload_above.frame}")
    print(f"reload_contact target xyz: {format_xyz(pose_xyz(reload_contact))} frame={reload_contact.frame}")

    diagnostic_position_limit = float(args.position_error_max_mm)
    planner_position_limit = max(1000.0, diagnostic_position_limit)
    hit_config["hit_action"]["approach_error_max_mm"] = planner_position_limit
    hit_config["hit_action"]["strike_error_max_mm"] = planner_position_limit
    hit_config["hit_action"]["reload_error_max_mm"] = planner_position_limit

    planner = HitTrajectoryPlanner(
        hit_config,
        controller_config,
        home_pose,
        reload_pose,
        ik_solver,
        safety_checker,
    )
    trajectory = []
    home_profile = phase_profile(hit_config, "return_home")
    home_pose_fk = ik_solver.fk_pose(home_angles, template)
    home_point = planner.exact_joint_point("home", home_pose_fk, home_angles, home_profile)
    trajectory.append(home_point)

    seed, reload_motion, reason = planner.build_press_motion(
        trajectory,
        "reload",
        reload_above,
        reload_contact,
        home_angles,
        reload_angles["gripper"],
        approach_phase="move_to_reload",
        press_phase="reload_hit_down",
        hold_phase="reload_hold",
        lift_phase="reload_hit_up",
        contact_dwell_key="hit_contact_dwell_s",
    )
    if reason:
        raise RuntimeError(f"core build_press_motion failed for reload diagnostic: {reason}")

    return_home_points = planner.copy_reversed_phase(
        reload_motion["approach_points"],
        "return_home",
        profile_name="return_home",
    )
    trajectory.extend(return_home_points)

    down_points = reload_motion["press_points"]
    down_debug_rows = []
    target_axis = np.array([0.0, 0.0, -1.0], dtype=float)
    orientation_mode = str(tool_config.get("orientation_mode", "Z"))
    for index, point in enumerate(down_points, start=1):
        matrix = fk_matrix(ik_solver, point.angles)
        actual_axis = tool_axis_from_matrix(matrix, orientation_mode)
        orient_error = point.orientation_error_deg
        if orient_error is None:
            orient_error = orientation_error_deg(matrix, orientation_mode, target_axis)
        achieved = point.achieved_position_m or [float("nan")] * 3
        down_debug_rows.append(
            {
                "index": index,
                "target": pose_xyz(point.pose),
                "achieved": achieved,
                "position_error_mm": float(point.position_error_mm),
                "orientation_error_deg": orient_error,
                "orientation_requested": bool(point.orientation_requested),
                "orientation_fallback": bool(point.orientation_fallback),
                "actual_tool_axis": actual_axis,
                "safety_reason": None,
            }
        )
        if float(point.position_error_mm) > diagnostic_position_limit:
            print(
                f"reload_hit_down #{index:02d} diagnostic warning: "
                f"IK position error {float(point.position_error_mm):.3f} mm > "
                f"{diagnostic_position_limit:.3f} mm"
            )

    contract = assert_reload_hit_contract(
        down_points,
        down_debug_rows,
        reload_above,
        reload_contact,
        max_position_error_mm=float(args.position_error_max_mm),
        max_orientation_error_deg=float(args.orientation_error_max_deg),
        strict_tool_down=bool(args.strict_tool_down),
    )

    return {
        "hit_config": hit_config,
        "controller_config": controller_config,
        "trajectory": trajectory,
        "home_angles": home_angles,
        "home_raw": home_raw,
        "reload_angles": reload_angles,
        "reload_raw": reload_raw,
        "reload_above": reload_above,
        "reload_contact": reload_contact,
        "contract": contract,
    }


def print_timeout_joint_errors(target_raw, current_raw):
    print("strict wait timeout joint errors:")
    ordered = FOCUS_JOINTS + [joint for joint in ACTIVE_JOINTS if joint not in FOCUS_JOINTS]
    for joint in ordered:
        target = int(target_raw[joint])
        current = int(current_raw.get(joint, -1))
        error = abs(target - current) if joint in current_raw else None
        print(f"  {joint:14s} target={target:5d} current={current:5d} error={error}")


def execute_strict(plan, args):
    controller_config = plan["controller_config"]
    hit_config = plan["hit_config"]
    trajectory = plan["trajectory"]
    action = hit_config["hit_action"]
    command_joints = list(action.get("command_joints", ACTIVE_JOINTS))
    use_sync_write = bool(action.get("sync_write", False))

    servo = ServoInterface(controller_config)
    servo.connect()
    try:
        state = servo.read_state()
        print_joint_table("current joint angles before execution:", state.angles, state.raw)
        for index, point in enumerate(trajectory, start=1):
            frame_joints = command_joints
            if use_sync_write:
                servo.send_point_sync(point, joints=frame_joints)
            else:
                servo.send_point(point, joints=frame_joints)
            target_raw = {joint: int(point.raw[joint]) for joint in frame_joints}
            reached, max_error, current_raw, wait_stats = servo.wait_until_reached(
                target_raw,
                tolerance_raw=int(args.wait_raw_tolerance),
                timeout_s=float(args.wait_timeout),
                poll_interval_s=float(args.wait_poll_interval),
                joints=frame_joints,
            )
            print(
                f"execute #{index:03d} phase={point.phase:16s} "
                f"reached={reached} max_raw_error={max_error} "
                f"wait_elapsed={wait_stats['elapsed_time']:.3f}s"
            )
            if not reached:
                print_timeout_joint_errors(target_raw, current_raw)
                raise RuntimeError(f"{point.phase} strict wait timeout; stopping immediately.")
            time.sleep(float(point.dt))
    finally:
        servo.close()


def main():
    args = build_parser().parse_args()
    plan = build_reload_diagnostic(args)
    contract = plan["contract"]
    print(
        "reload_hit_contract_result: "
        f"ok={contract['ok']} "
        f"max_position_error_mm={contract['max_position_error_mm']:.3f} "
        f"max_orientation_error_deg={contract['max_orientation_error_deg']:.3f} "
        f"orientation_ok={contract['orientation_ok']} "
        f"strict_tool_down={contract['strict_tool_down']}"
    )
    if not contract["orientation_ok"] and not contract["strict_tool_down"]:
        print("提示：当前与 hit 使用同样的 position-first fallback；位置可达时允许姿态回退，但会打印姿态误差。")
    if not contract["ok"]:
        print("诊断结论：reload_hit_down 坐标合同失败。不要继续跑完整 hit 流程。")
        print("当前含义：目标轨迹已定义，但 IK achieved_position 或姿态约束未满足阈值。")
        if args.yes:
            raise SystemExit("拒绝执行：reload_hit_down contract failed.")
        return
    if not args.yes:
        print("dry-run only. Add --yes to execute the reload diagnostic trajectory.")
        return
    execute_strict(plan, args)


if __name__ == "__main__":
    main()
