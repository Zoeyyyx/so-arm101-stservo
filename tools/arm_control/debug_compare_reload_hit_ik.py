"""Compare reload and hit IK/Cartesian planning with the same target.

This script does not execute servos. It builds one reload press motion and one
hit press motion from the same seed and the same Cartesian above/contact poses,
then prints the actual IKSolver.solve inputs and outputs for both paths.
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ARM_CONTROL_ROOT = REPO_ROOT / "tools" / "arm_control"
STSERVO_ROOT = REPO_ROOT / "tools" / "stservo"
for path in (ARM_CONTROL_ROOT, STSERVO_ROOT):
    text = str(path)
    if text not in sys.path:
        sys.path.insert(0, text)

from core.arm_controller import ArmController  # noqa: E402
from core.servo_interface import target_raw_from_angles  # noqa: E402
from core.trajectory_planner import phase_ik_name, phase_profile, phase_profile_name  # noqa: E402
from core.types import ALL_JOINTS, Pose6D  # noqa: E402


DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"
DEFAULT_READY_CONFIG = REPO_ROOT / "config" / "ready_pose.json"


def build_parser():
    parser = argparse.ArgumentParser(description="Compare reload_hit_down and hit_down IK paths.")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG))
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument("--ready-config", default=str(DEFAULT_READY_CONFIG))
    parser.add_argument("--x", type=float, default=None)
    parser.add_argument("--y", type=float, default=None)
    parser.add_argument("--contact-z", type=float, default=0.010)
    parser.add_argument("--above-z", type=float, default=None)
    parser.add_argument("--frame", default="so101_base")
    parser.add_argument("--max-rows", type=int, default=999)
    return parser


def fmt_pose(pose):
    return f"x={pose.x:.6f} y={pose.y:.6f} z={pose.z:.6f} rpy=({pose.roll:.3f},{pose.pitch:.3f},{pose.yaw:.3f})"


def fmt_xyz(values):
    if values is None:
        return "None"
    return f"x={float(values[0]):.6f} y={float(values[1]):.6f} z={float(values[2]):.6f}"


def max_abs_joint_delta(a, b):
    return max(abs(float(a[joint]) - float(b[joint])) for joint in ALL_JOINTS)


def raw_dict(angles, controller_config):
    return target_raw_from_angles(angles, controller_config)


def print_phase_config(controller, phase):
    profile = phase_profile(controller.hit_config, phase)
    print(
        f"  {phase:18s} profile={phase_profile_name(controller.hit_config, phase):24s} "
        f"ik_phase={phase_ik_name(phase):22s} steps={profile.steps:3d} "
        f"speed={profile.speed:3d} acc={profile.acc:3d} dt={profile.dt:.3f}"
    )


def capture_solver_calls(controller):
    logs = []
    original_solve = controller.planner.ik_solver.solve

    def wrapped_solve(pose, seed_angles, hit_config, gripper_target, phase_name, max_iter=None, force_position_only=False):
        record = {
            "pose": copy.deepcopy(pose),
            "seed": dict(seed_angles),
            "phase_name": phase_name,
            "max_iter": max_iter,
            "force_position_only": bool(force_position_only),
        }
        try:
            angles, debug = original_solve(
                pose,
                seed_angles,
                hit_config,
                gripper_target,
                phase_name,
                max_iter=max_iter,
                force_position_only=force_position_only,
            )
        except Exception as exc:  # noqa: BLE001
            record["error"] = str(exc)
            logs.append(record)
            raise
        record["angles"] = dict(angles)
        record["raw"] = raw_dict(angles, controller.controller_config)
        record["debug"] = copy.deepcopy(debug)
        logs.append(record)
        return angles, debug

    controller.planner.ik_solver.solve = wrapped_solve
    return logs, original_solve


def run_motion(controller, label, above_pose, contact_pose, seed, gripper_target):
    phase_sets = {
        "reload": {
            "approach_phase": "move_to_reload",
            "press_phase": "reload_hit_down",
            "hold_phase": "reload_hold",
            "lift_phase": "reload_hit_up",
        },
        "hit": {
            "approach_phase": "move_to_hit_above",
            "press_phase": "hit_down",
            "hold_phase": "hit_hold",
            "lift_phase": "hit_up",
        },
    }
    phases = phase_sets[label]
    trajectory = []
    logs, original_solve = capture_solver_calls(controller)
    try:
        end_seed, motion, reason = controller.planner.build_press_motion(
            trajectory,
            label,
            copy.deepcopy(above_pose),
            copy.deepcopy(contact_pose),
            dict(seed),
            gripper_target,
            contact_dwell_key="hit_contact_dwell_s",
            **phases,
        )
    finally:
        controller.planner.ik_solver.solve = original_solve

    return {
        "label": label,
        "phases": phases,
        "trajectory": trajectory,
        "logs": logs,
        "end_seed": end_seed,
        "motion": motion,
        "reason": reason,
    }


def print_solver_logs(result, max_rows):
    print(f"\n{result['label']} IKSolver.solve calls:")
    for index, row in enumerate(result["logs"][:max_rows], start=1):
        debug = row.get("debug", {})
        seed = row["seed"]
        print(
            f"  #{index:02d} phase={row['phase_name']} force_position_only={row['force_position_only']} "
            f"max_iter={row['max_iter']} target={fmt_pose(row['pose'])}"
        )
        print(
            "      seed "
            + " ".join(f"{joint}={float(seed[joint]):.3f}" for joint in ALL_JOINTS)
        )
        if "error" in row:
            print(f"      ERROR {row['error']}")
            continue
        print(
            f"      achieved={fmt_xyz(debug.get('achieved_position'))} "
            f"position_error_mm={float(debug.get('position_error_mm', 0.0)):.6f} "
            f"orientation_error_deg={debug.get('orientation_error_deg')} "
            f"orientation_requested={debug.get('orientation_requested')} "
            f"orientation_fallback={debug.get('orientation_fallback')}"
        )
        print(
            "      raw "
            + " ".join(f"{joint}={int(row['raw'][joint])}" for joint in ALL_JOINTS)
        )


def compare_press_points(reload_result, hit_result, max_rows):
    reload_points = reload_result["motion"]["press_points"] if reload_result["motion"] else []
    hit_points = hit_result["motion"]["press_points"] if hit_result["motion"] else []
    count = min(len(reload_points), len(hit_points), max_rows)
    print("\npress point comparison:")
    print(f"  reload_points={len(reload_points)} hit_points={len(hit_points)} compared={count}")
    for index in range(count):
        reload_point = reload_points[index]
        hit_point = hit_points[index]
        pose_delta = max(
            abs(reload_point.pose.x - hit_point.pose.x),
            abs(reload_point.pose.y - hit_point.pose.y),
            abs(reload_point.pose.z - hit_point.pose.z),
        )
        joint_delta = max_abs_joint_delta(reload_point.angles, hit_point.angles)
        raw_delta = max(abs(int(reload_point.raw[joint]) - int(hit_point.raw[joint])) for joint in ALL_JOINTS)
        achieved_delta = None
        if reload_point.achieved_position_m is not None and hit_point.achieved_position_m is not None:
            achieved_delta = max(
                abs(float(reload_point.achieved_position_m[i]) - float(hit_point.achieved_position_m[i]))
                for i in range(3)
            )
        print(
            f"  #{index + 1:02d} pose_delta_m={pose_delta:.9f} "
            f"achieved_delta_m={achieved_delta if achieved_delta is not None else 'None'} "
            f"max_joint_delta_deg={joint_delta:.9f} max_raw_delta={raw_delta}"
        )
        print(
            f"      reload target={fmt_pose(reload_point.pose)} achieved={fmt_xyz(reload_point.achieved_position_m)} "
            f"pos_err={reload_point.position_error_mm:.6f} orient_err={reload_point.orientation_error_deg}"
        )
        print(
            f"      hit    target={fmt_pose(hit_point.pose)} achieved={fmt_xyz(hit_point.achieved_position_m)} "
            f"pos_err={hit_point.position_error_mm:.6f} orient_err={hit_point.orientation_error_deg}"
        )


def main():
    args = build_parser().parse_args()
    controller = ArmController.from_files(args.hit_config, args.home_config, args.ready_config)
    action = controller.hit_config["hit_action"]
    reload_config = action.get("reload_pose", {})

    x = float(args.x if args.x is not None else reload_config.get("x_m", 0.249))
    y = float(args.y if args.y is not None else reload_config.get("y_m", 0.085))
    above_z = args.above_z
    if above_z is None:
        above_z = float(args.contact_z) + float(action.get("strike_height_m", 0.10))

    above_pose = Pose6D(x, y, float(above_z), 0.0, 0.0, 0.0, args.frame)
    contact_pose = Pose6D(x, y, float(args.contact_z), 0.0, 0.0, 0.0, args.frame)
    home_angles = controller.planner.home_angles(
        controller.home_pose["joint_angles_deg"],
        action.get("default_gripper", controller.home_pose["joint_angles_deg"]["gripper"]),
    )
    ready_angles = controller.planner.ready_angles(
        home_angles,
        action.get("default_gripper", home_angles["gripper"]),
    )
    gripper_target = float(ready_angles["gripper"])

    print("debug_compare_reload_hit_ik")
    print(f"  hit_config={Path(args.hit_config).resolve()}")
    print(f"  home_config={Path(args.home_config).resolve()}")
    print(f"  ready_config={Path(args.ready_config).resolve()}")
    print(f"  above_pose={fmt_pose(above_pose)}")
    print(f"  contact_pose={fmt_pose(contact_pose)}")
    print("  seed=ready_angles for both reload and hit")
    print("  tool_orientation:")
    tool_config = controller.hit_config["tool_orientation"]
    print(f"    enforce_tool_down={tool_config.get('enforce_tool_down')}")
    print(f"    tool_down_phases={tool_config.get('tool_down_phases')}")
    print(f"    orientation_mode={tool_config.get('orientation_mode')}")
    print(f"    target_axis_in_base={tool_config.get('target_axis_in_base')}")
    print(f"    max_orientation_error_deg={tool_config.get('max_orientation_error_deg')}")
    print(f"    position_first_fallback={tool_config.get('position_first_fallback')}")
    print("  phase configs:")
    for phase in (
        "move_to_reload",
        "move_to_hit_above",
        "reload_hit_down",
        "hit_down",
        "reload_hold",
        "hit_hold",
        "reload_hit_up",
        "hit_up",
    ):
        print_phase_config(controller, phase)

    reload_result = run_motion(controller, "reload", above_pose, contact_pose, ready_angles, gripper_target)
    hit_result = run_motion(controller, "hit", above_pose, contact_pose, ready_angles, gripper_target)

    for result in (reload_result, hit_result):
        print(f"\n{result['label']} result:")
        print(f"  reason={result['reason']}")
        print(f"  trajectory_points={len(result['trajectory'])}")
        if result["motion"]:
            print(
                f"  approach_points={len(result['motion']['approach_points'])} "
                f"press_points={len(result['motion']['press_points'])} "
                f"hold_points={len(result['motion']['hold_points'])} "
                f"lift_points={len(result['motion']['lift_points'])}"
            )
        print_solver_logs(result, args.max_rows)

    compare_press_points(reload_result, hit_result, args.max_rows)

    reload_ok = reload_result["reason"] is None
    hit_ok = hit_result["reason"] is None
    if hit_ok and not reload_ok:
        raise SystemExit("hit succeeded but reload failed: reload path still differs.")
    if reload_ok and hit_ok:
        print("\ncomparison_result: both paths succeeded.")
    else:
        print("\ncomparison_result: at least one path failed; inspect reason/logs above.")


if __name__ == "__main__":
    main()
