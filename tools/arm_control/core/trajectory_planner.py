"""打靶与回 home 轨迹规划。"""

from __future__ import annotations

import copy
import numpy as np

from core.servo_interface import target_raw_from_angles
from core.types import ACTIVE_JOINTS, ALL_JOINTS, PASSIVE_JOINTS, MotionProfile, PlanResult, Pose6D, TrajectoryPoint


PHASE_PROFILE_ALIASES = {
    "move_to_reload": "approach_above_target",
    "move_to_hit_above": "approach_above_target",
    "move_to_target_above": "approach_above_target",
    "reload_hit_down": "strike_down",
    "reload_hold": "hit_hold",
    "reload_hit_up": "return_strike_down",
    "hit_down": "strike_down",
    "hit_up": "return_strike_down",
    "target_hit_down": "strike_down",
    "target_hit_up": "return_strike_down",
    "return_to_ready": "return_approach_above_target",
    "return_to_reload": "return_approach_above_target",
}

PHASE_IK_ALIASES = {
    "move_to_reload": "approach_above_target",
    "move_to_hit_above": "approach_above_target",
    "move_to_target_above": "approach_above_target",
    "reload_hit_down": "strike_down",
    "hit_down": "strike_down",
    "target_hit_down": "strike_down",
}


def phase_profile_name(hit_config, phase_name):
    phases = hit_config["hit_action"]["phases"]
    alias = PHASE_PROFILE_ALIASES.get(phase_name)
    if alias in phases:
        return alias
    if phase_name in phases:
        return phase_name
    return phase_name


def phase_ik_name(phase_name):
    return PHASE_IK_ALIASES.get(phase_name, phase_name)


def phase_profile(hit_config, phase_name) -> MotionProfile:
    """读取一个阶段的运动参数。"""
    profile = hit_config["hit_action"]["phases"][phase_profile_name(hit_config, phase_name)]
    action = hit_config.get("hit_action", {})
    dt = float(profile["dt"])
    if int(profile["speed"]) <= int(action.get("low_speed_dt_threshold", 120)):
        dt = max(dt, float(action.get("min_low_speed_dt_s", 0.06)))
    return MotionProfile(
        steps=int(profile["steps"]),
        speed=int(profile["speed"]),
        acc=int(profile["acc"]),
        dt=dt,
    )


def phase_position_error_limit_mm(hit_config, controller_config, phase_name):
    """按阶段读取允许 IK 位置误差，单位 mm。"""
    action = hit_config.get("hit_action", {})
    default_limit = float(controller_config["workspace"].get("position_error_max_mm", 10.0))
    profile_name = phase_profile_name(hit_config, phase_name)
    if profile_name == "approach_above_target":
        return float(action.get("approach_error_max_mm", default_limit))
    if profile_name == "strike_down":
        return float(action.get("strike_error_max_mm", default_limit))
    if profile_name in {"move_to_reload", "reload_press_down", "reload_lift_up"}:
        return float(action.get("reload_error_max_mm", action.get("approach_error_max_mm", default_limit)))
    return default_limit


def smoothstep(t):
    """三次 smoothstep 插值，起止更柔和。"""
    t = float(np.clip(t, 0.0, 1.0))
    return 3.0 * t * t - 2.0 * t * t * t


def max_joint_delta_degrees(start_angles, end_angles):
    """计算两组关节角的最大单关节变化。"""
    return max(abs(float(end_angles[joint]) - float(start_angles[joint])) for joint in ACTIVE_JOINTS)


def max_adjacent_joint_delta(points):
    """计算轨迹相邻点最大单关节变化。"""
    max_delta = 0.0
    for previous, current in zip(points, points[1:]):
        max_delta = max(max_delta, max_joint_delta_degrees(previous.angles, current.angles))
    return max_delta


def max_phase_joint_delta(start_angles, points):
    """计算从阶段起点到该阶段所有轨迹点之间的最大主动关节跳变。"""
    max_delta = 0.0
    previous_angles = start_angles
    for point in points:
        max_delta = max(max_delta, max_joint_delta_degrees(previous_angles, point.angles))
        previous_angles = point.angles
    return max_delta


def max_raw_delta_between(previous_raw, current_raw, joints=ACTIVE_JOINTS):
    """计算两个 raw 目标之间主动关节的最大变化。"""
    return max(abs(int(current_raw[joint]) - int(previous_raw[joint])) for joint in joints)


def compare_reversed_points(forward_points, reverse_points, joints=ALL_JOINTS):
    """逐点检查 reverse_points 是否等于 forward_points 的反序。

    比较对象包括：
    - 关节角 angles；
    - 舵机 raw 目标。
    """
    result = {
        "ok": True,
        "forward_points": len(forward_points),
        "reverse_points": len(reverse_points),
        "max_angle_error_deg": 0.0,
        "max_raw_error": 0,
        "max_pose_error_m": 0.0,
        "first_mismatch": None,
    }
    if len(forward_points) != len(reverse_points):
        result["ok"] = False
        result["first_mismatch"] = {
            "reason": "point_count_mismatch",
            "forward_points": len(forward_points),
            "reverse_points": len(reverse_points),
        }
        return result

    for index, (forward, returned) in enumerate(zip(reversed(forward_points), reverse_points), start=1):
        for joint in joints:
            angle_error = abs(float(forward.angles[joint]) - float(returned.angles[joint]))
            raw_error = abs(int(forward.raw[joint]) - int(returned.raw[joint]))
            result["max_angle_error_deg"] = max(result["max_angle_error_deg"], angle_error)
            result["max_raw_error"] = max(result["max_raw_error"], raw_error)
            if (angle_error > 1e-9 or raw_error != 0) and result["first_mismatch"] is None:
                result["ok"] = False
                result["first_mismatch"] = {
                    "index": index,
                    "joint": joint,
                    "forward_phase": forward.phase,
                    "reverse_phase": returned.phase,
                    "forward_angle": float(forward.angles[joint]),
                    "reverse_angle": float(returned.angles[joint]),
                    "angle_error_deg": angle_error,
                    "forward_raw": int(forward.raw[joint]),
                    "reverse_raw": int(returned.raw[joint]),
                    "raw_error": raw_error,
                }
        pose_errors = [
            abs(float(forward.pose.x) - float(returned.pose.x)),
            abs(float(forward.pose.y) - float(returned.pose.y)),
            abs(float(forward.pose.z) - float(returned.pose.z)),
            abs(float(forward.pose.roll) - float(returned.pose.roll)),
            abs(float(forward.pose.pitch) - float(returned.pose.pitch)),
            abs(float(forward.pose.yaw) - float(returned.pose.yaw)),
        ]
        max_pose_error = max(pose_errors)
        result["max_pose_error_m"] = max(result["max_pose_error_m"], max_pose_error)
        if (max_pose_error > 1e-12 or forward.pose.frame != returned.pose.frame) and result["first_mismatch"] is None:
            result["ok"] = False
            result["first_mismatch"] = {
                "index": index,
                "reason": "pose_mismatch",
                "forward_phase": forward.phase,
                "reverse_phase": returned.phase,
                "forward_pose": vars(forward.pose),
                "reverse_pose": vars(returned.pose),
                "max_pose_error": max_pose_error,
            }
        if not result["ok"]:
            break
    return result


def verify_strict_reverse_return(outbound_points, strike_points, rebound_points, return_points):
    """验证回程轨迹是否严格等于去程轨迹反序。"""
    strike_check = compare_reversed_points(strike_points, rebound_points)
    outbound_check = compare_reversed_points(outbound_points, return_points)
    ok = bool(strike_check["ok"] and outbound_check["ok"])
    reason = ""
    if not ok:
        reason = "回程轨迹数据结构不是去程轨迹的严格反序。"
    return {
        "ok": ok,
        "reason": reason,
        "strike_down_vs_return_strike_down": strike_check,
        "home_approach_vs_return_stack": outbound_check,
    }


def pose_xyz(pose):
    """取末端位姿的位置三元组。"""
    return float(pose.x), float(pose.y), float(pose.z)


def assert_strike_cartesian_contract(strike_points, target_pose_base, poses, hit_config, controller_config):
    """校验 strike_down 是否严格符合输入靶心坐标合同。

    合同：
    - above_target = (target_x, target_y, target_z + strike_height)
    - contact = (target_x, target_y, target_z + contact_offset)
    - strike_down 阶段 x/y 恒定，z 单调下降
    - 首点是 above_target，末点是 contact
    - IK achieved_position 误差不超过配置允许值
    """
    tolerance_m = 1e-9
    max_position_error = phase_position_error_limit_mm(hit_config, controller_config, "strike_down")
    action = hit_config["hit_action"]
    strike_height = max(
        float(action.get("strike_height_m", action.get("above_target_height_m", action.get("hover_height_m", 0.08)))),
        float(action.get("min_strike_height_m", 0.06)),
    )
    contact_offset = float(action["contact_offset_m"])
    expected_above = (
        float(target_pose_base.x),
        float(target_pose_base.y),
        float(target_pose_base.z) + strike_height,
    )
    expected_contact = (
        float(target_pose_base.x),
        float(target_pose_base.y),
        float(target_pose_base.z) + contact_offset,
    )
    result = {
        "ok": True,
        "reason": "",
        "strike_points": len(strike_points),
        "expected_above": expected_above,
        "expected_contact": expected_contact,
        "max_position_error_mm": 0.0,
        "max_xy_error_m": 0.0,
        "first_mismatch": None,
    }

    def fail(reason, detail=None):
        result["ok"] = False
        result["reason"] = reason
        result["first_mismatch"] = detail
        return result

    if not strike_points:
        return fail("strike_down 轨迹为空。")

    above_xyz = pose_xyz(poses["above_target"])
    contact_xyz = pose_xyz(poses["contact"])
    for label, actual, expected in [
        ("above_target", above_xyz, expected_above),
        ("contact", contact_xyz, expected_contact),
    ]:
        errors = [abs(actual[index] - expected[index]) for index in range(3)]
        if max(errors) > tolerance_m:
            return fail(
                f"{label} 不符合输入坐标合同。",
                {"label": label, "actual": actual, "expected": expected, "errors": errors},
            )

    first_xyz = pose_xyz(strike_points[0].pose)
    last_xyz = pose_xyz(strike_points[-1].pose)
    for label, actual, expected in [
        ("strike_down 首点", first_xyz, expected_above),
        ("strike_down 末点", last_xyz, expected_contact),
    ]:
        errors = [abs(actual[index] - expected[index]) for index in range(3)]
        if max(errors) > tolerance_m:
            return fail(
                f"{label} 不符合 above/contact。",
                {"label": label, "actual": actual, "expected": expected, "errors": errors},
            )

    previous_z = None
    for index, point in enumerate(strike_points, start=1):
        x_error = abs(float(point.pose.x) - float(target_pose_base.x))
        y_error = abs(float(point.pose.y) - float(target_pose_base.y))
        result["max_xy_error_m"] = max(result["max_xy_error_m"], x_error, y_error)
        if x_error > tolerance_m or y_error > tolerance_m:
            return fail(
                "strike_down 阶段 x/y 没有恒等于 target_x/target_y。",
                {
                    "index": index,
                    "pose": pose_xyz(point.pose),
                    "target_xy": (float(target_pose_base.x), float(target_pose_base.y)),
                    "x_error": x_error,
                    "y_error": y_error,
                },
            )
        current_z = float(point.pose.z)
        if previous_z is not None and current_z > previous_z + tolerance_m:
            return fail(
                "strike_down 阶段 z 不是单调下降。",
                {"index": index, "previous_z": previous_z, "current_z": current_z},
            )
        previous_z = current_z
        result["max_position_error_mm"] = max(result["max_position_error_mm"], float(point.position_error_mm))
        if float(point.position_error_mm) > max_position_error:
            return fail(
                "strike_down IK achieved_position 误差超过允许范围。",
                {
                    "index": index,
                    "target_pose": pose_xyz(point.pose),
                    "achieved_position_m": point.achieved_position_m,
                    "position_error_mm": float(point.position_error_mm),
                    "max_position_error_mm": max_position_error,
                },
            )
    return result


def adaptive_steps(base_steps, start_angles, end_angles, hit_config):
    """根据关节角变化自动增加步数。"""
    max_step = float(hit_config["hit_action"].get("max_joint_step_deg", 4.0))
    if max_step <= 0:
        return max(1, int(base_steps))
    needed_steps = int(np.ceil(max_joint_delta_degrees(start_angles, end_angles) / max_step))
    return max(1, int(base_steps), needed_steps)


def interpolate_joint_angles_smooth(current_angles, target_angles, steps):
    """关节空间 smoothstep 插值。"""
    path = []
    steps = max(1, int(steps))
    for step in range(1, steps + 1):
        ratio = smoothstep(step / steps)
        point = {}
        for joint in ALL_JOINTS:
            start = float(current_angles[joint])
            end = float(target_angles[joint])
            point[joint] = start + (end - start) * ratio
        path.append(point)
    return path


def interpolate_pose_xyz(start_pose, end_pose, steps):
    """末端笛卡尔直线 smoothstep 插值。"""
    points = []
    steps = max(1, int(steps))
    for step in range(1, steps + 1):
        ratio = smoothstep(step / steps)
        points.append(
            Pose6D(
                x=start_pose.x + (end_pose.x - start_pose.x) * ratio,
                y=start_pose.y + (end_pose.y - start_pose.y) * ratio,
                z=start_pose.z + (end_pose.z - start_pose.z) * ratio,
                roll=start_pose.roll,
                pitch=start_pose.pitch,
                yaw=start_pose.yaw,
                frame=start_pose.frame,
            )
        )
    return points


def interpolate_pose_z(start_pose, end_pose, steps):
    """只沿 Z 方向插值，保持 X/Y 不变，并包含首尾点。"""
    points = []
    steps = max(2, int(steps))
    for step in range(steps):
        ratio = smoothstep(step / (steps - 1))
        points.append(
            Pose6D(
                x=start_pose.x,
                y=start_pose.y,
                z=start_pose.z + (end_pose.z - start_pose.z) * ratio,
                roll=start_pose.roll,
                pitch=start_pose.pitch,
                yaw=start_pose.yaw,
                frame=start_pose.frame,
            )
        )
    return points


class HitTrajectoryPlanner:
    """规划 home 出发、下击、按原路反向返回 home 的轨迹。"""

    def __init__(self, hit_config, controller_config, home_pose, ready_pose, ik_solver, safety_checker):
        self.hit_config = hit_config
        self.controller_config = controller_config
        self.home_pose = home_pose
        self.ready_pose = ready_pose
        self.ik_solver = ik_solver
        self.safety_checker = safety_checker

    def home_tolerance_deg(self):
        """启动姿态必须接近 home 的容差。"""
        return float(self.home_pose.get("start_tolerance_deg", 8.0))

    def home_angles(self, current_angles, gripper_target):
        """从 home_pose.json 读取配置 home 角度。"""
        return self.configured_joint_angles(self.home_pose, current_angles, gripper_target)

    def ready_angles(self, current_angles, gripper_target):
        """从 ready_pose.json 读取安全展开关节角。"""
        return self.configured_joint_angles(self.ready_pose, current_angles, gripper_target)

    def configured_joint_angles(self, pose_config, current_angles, gripper_target):
        """从某个姿态配置中读取关节角，未配置的关节保持当前值。"""
        configured = pose_config.get("joint_angles_deg", {})
        angles = dict(current_angles)
        for joint in ALL_JOINTS:
            if joint in PASSIVE_JOINTS:
                angles[joint] = float(current_angles[joint])
                continue
            if joint in configured:
                angles[joint] = float(configured[joint])
        return angles

    def build_hit_poses(self, target_pose_base):
        """构造靶位上方点和接触点。"""
        action = self.hit_config["hit_action"]
        configured_height = float(
            action.get("strike_height_m", action.get("above_target_height_m", action.get("hover_height_m", 0.08)))
        )
        min_height = float(action.get("min_strike_height_m", 0.06))
        above_height = max(configured_height, min_height)
        above_target_z = target_pose_base.z + above_height
        contact_z = target_pose_base.z + float(action["contact_offset_m"])

        def pose_at(z_value):
            return Pose6D(
                x=target_pose_base.x,
                y=target_pose_base.y,
                z=float(z_value),
                roll=target_pose_base.roll,
                pitch=target_pose_base.pitch,
                yaw=target_pose_base.yaw,
                frame=target_pose_base.frame,
            )

        return {"above_target": pose_at(above_target_z), "contact": pose_at(contact_z)}

    def build_reload_target_pose(self, target_pose_base):
        """构造 reload 目标接触点。

        reload 现在复用 hit 的 above/contact 生成逻辑：这里仅提供目标平面坐标，
        build_hit_poses() 会生成 reload_above 和 reload_contact。
        """
        reload_config = self.hit_config["hit_action"].get("reload_pose", {})
        if not reload_config or not bool(reload_config.get("enabled", False)):
            return None

        if reload_config.get("z_m") is None:
            action = self.hit_config["hit_action"]
            configured_height = float(
                action.get("strike_height_m", action.get("above_target_height_m", action.get("hover_height_m", 0.08)))
            )
            min_height = float(action.get("min_strike_height_m", 0.06))
            z_value = float(target_pose_base.z) + max(configured_height, min_height)
        else:
            z_value = float(reload_config["z_m"])

        orientation_source = str(reload_config.get("orientation_source", "hit")).lower()
        if orientation_source == "hit":
            roll = float(target_pose_base.roll)
            pitch = float(target_pose_base.pitch)
            yaw = float(target_pose_base.yaw)
        else:
            roll = float(reload_config.get("roll", target_pose_base.roll))
            pitch = float(reload_config.get("pitch", target_pose_base.pitch))
            yaw = float(reload_config.get("yaw", target_pose_base.yaw))

        return Pose6D(
            x=float(reload_config.get("x_m", 0.249)),
            y=float(reload_config.get("y_m", 0.085)),
            z=z_value,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            frame=str(reload_config.get("frame", target_pose_base.frame)),
        )

    def build_reload_contact_pose(self, reload_above_pose):
        action = self.hit_config["hit_action"]
        reload_config = action.get("reload_pose", {})
        depth_value = reload_config.get("reload_hit_depth_m", reload_config.get("press_depth_m"))
        if depth_value is None:
            depth_value = action.get(
                "reload_hit_depth_m",
                action.get("strike_height_m", action.get("above_target_height_m", action.get("hover_height_m", 0.08))),
            )
        min_depth = float(reload_config.get("min_press_depth_m", action.get("min_strike_height_m", 0.06)))
        reload_hit_depth = max(float(depth_value), min_depth)
        return Pose6D(
            x=float(reload_above_pose.x),
            y=float(reload_above_pose.y),
            z=float(reload_above_pose.z) - reload_hit_depth,
            roll=float(reload_above_pose.roll),
            pitch=float(reload_above_pose.pitch),
            yaw=float(reload_above_pose.yaw),
            frame=reload_above_pose.frame,
        )

    def make_point(self, phase_name, pose, angles, debug, profile):
        """生成轨迹点。"""
        return TrajectoryPoint(
            phase=phase_name,
            pose=pose,
            angles=dict(angles),
            raw=target_raw_from_angles(angles, self.controller_config),
            position_error_mm=float(debug.get("position_error_mm", 0.0)),
            achieved_position_m=debug.get("achieved_position"),
            orientation_error_deg=debug.get("orientation_error_deg"),
            orientation_requested=bool(debug.get("orientation_requested", False)),
            orientation_fallback=bool(debug.get("orientation_fallback", False)),
            speed=int(profile.speed),
            acc=int(profile.acc),
            dt=float(profile.dt),
        )

    def exact_joint_point(self, phase_name, pose, angles, profile):
        """生成不需要 IK 的精确关节点。"""
        return self.make_point(
            phase_name,
            pose,
            angles,
            {
                "position_error_mm": 0.0,
                "orientation_error_deg": None,
                "orientation_requested": False,
                "orientation_fallback": False,
            },
            profile,
        )

    def append_joint_phase(self, trajectory, phase_name, start_angles, target_angles, pose_template):
        """关节空间插值阶段，不做笛卡尔 IK。"""
        profile = phase_profile(self.hit_config, phase_name)
        steps = adaptive_steps(int(profile.steps), start_angles, target_angles, self.hit_config)
        min_raw_step = int(self.hit_config["hit_action"].get("min_joint_raw_step", 4))
        allow_outside_safe = phase_name in set(
            self.hit_config["hit_action"].get("allow_outside_safe_phases", [])
        )
        current_seed = dict(start_angles)
        previous_raw = target_raw_from_angles(start_angles, self.controller_config)
        path = interpolate_joint_angles_smooth(start_angles, target_angles, steps)
        for point_index, angles in enumerate(path, start=1):
            joint_violations = [] if allow_outside_safe else self.safety_checker.validate_target_angles(angles)
            if joint_violations:
                detail = "\n".join(
                    f"  {joint}: target={value:.3f} deg, safe={low:.3f}..{high:.3f} deg"
                    for joint, value, low, high in joint_violations
                )
                return current_seed, f"{phase_name} 第 {point_index} 个关节插值点越界：\n{detail}"
            point_pose = self.ik_solver.fk_pose(angles, pose_template)
            point = self.exact_joint_point(phase_name, point_pose, angles, profile)
            is_last_point = point_index == len(path)
            raw_delta = max_raw_delta_between(previous_raw, point.raw)
            if is_last_point or min_raw_step <= 0 or raw_delta >= min_raw_step:
                trajectory.append(point)
                previous_raw = point.raw
            current_seed = angles
        return current_seed, None

    def append_cartesian_phase(self, trajectory, phase_name, poses, seed_angles, gripper_target):
        """逐点 IK 追加笛卡尔阶段。"""
        profile = phase_profile(self.hit_config, phase_name)
        current_seed = dict(seed_angles)
        phase_start = len(trajectory)
        max_error_mm = phase_position_error_limit_mm(self.hit_config, self.controller_config, phase_name)
        default_max_iter = int(self.controller_config["ik"].get("max_iter", 200))
        min_raw_step = int(self.hit_config["hit_action"].get("min_joint_raw_step", 4))
        previous_raw = target_raw_from_angles(seed_angles, self.controller_config)
        solver_phase_name = phase_ik_name(phase_name)
        force_position_only = solver_phase_name in {"approach_above_target"}
        for point_index, pose in enumerate(poses, start=1):
            last_success = trajectory[-1].angles if len(trajectory) > phase_start else current_seed
            attempts = [
                ("current_seed", current_seed, default_max_iter),
                ("last_success_seed", last_success, default_max_iter),
                ("last_success_seed_more_iter", last_success, default_max_iter * 3),
            ]
            best = None
            errors = []
            for attempt_name, attempt_seed, attempt_iter in attempts:
                try:
                    candidate_angles, candidate_debug = self.ik_solver.solve(
                        pose,
                        attempt_seed,
                        self.hit_config,
                        gripper_target,
                        solver_phase_name,
                        max_iter=attempt_iter,
                        force_position_only=force_position_only,
                    )
                except Exception as exc:  # noqa: BLE001 - 返回 PlanResult 给上层处理。
                    errors.append({"attempt": attempt_name, "error": str(exc)})
                    continue
                candidate_debug["attempt"] = attempt_name
                candidate_debug["point_index"] = point_index
                if best is None or candidate_debug["position_error_mm"] < best[1]["position_error_mm"]:
                    best = (candidate_angles, candidate_debug)
                if candidate_debug["position_error_mm"] <= max_error_mm:
                    best = (candidate_angles, candidate_debug)
                    break

            if best is None:
                return current_seed, f"{phase_name} 第 {point_index} 点 IK 求解失败：{errors}"
            angles, debug = best
            if debug["position_error_mm"] > max_error_mm:
                next_pose = poses[point_index] if point_index < len(poses) else None
                local_errors = self._local_ik_error_context(
                    trajectory,
                    phase_start,
                    point_index,
                    debug,
                    next_pose,
                    current_seed,
                    gripper_target,
                    phase_name,
                )
                return current_seed, (
                    f"{phase_name} 第 {point_index} 点 IK 位置误差过大："
                    f"{debug['position_error_mm']:.3f} mm > {max_error_mm:.3f} mm\n"
                    f"  attempt={debug.get('attempt')} max_iter={debug.get('max_iter')}\n"
                    f"  target: x={pose.x:.4f} y={pose.y:.4f} z={pose.z:.4f}\n"
                    f"  achieved={debug.get('achieved_position')}\n"
                    f"  local_errors={local_errors}"
                )

            reason = self.safety_checker.validate_solution(
                phase_name,
                point_index,
                pose,
                angles,
                debug,
                max_position_error_mm=max_error_mm,
            )
            if reason:
                return current_seed, reason
            point = self.make_point(phase_name, pose, angles, debug, profile)
            is_boundary_point = point_index == 1 or point_index == len(poses)
            raw_delta = max_raw_delta_between(previous_raw, point.raw)
            if is_boundary_point or min_raw_step <= 0 or raw_delta >= min_raw_step:
                trajectory.append(point)
                previous_raw = point.raw
            current_seed = angles
        return current_seed, None

    def append_adaptive_cartesian_phase(self, trajectory, phase_name, poses_factory, seed_angles, gripper_target):
        """追加笛卡尔阶段；若关节跳变过大，自动加密中间点后重算。"""
        profile = phase_profile(self.hit_config, phase_name)
        action = self.hit_config["hit_action"]
        max_step_delta = float(action.get("max_joint_step_deg", 4.0))
        max_steps = int(action.get("max_cartesian_phase_steps", 80))
        steps = max(1, int(profile.steps))
        start_len = len(trajectory)
        last_reason = None
        diagnostics = []

        while True:
            del trajectory[start_len:]
            current_seed, reason = self.append_cartesian_phase(
                trajectory,
                phase_name,
                poses_factory(steps),
                seed_angles,
                gripper_target,
            )
            phase_points = list(trajectory[start_len:])
            if reason:
                last_reason = reason
                break

            observed_delta = max_phase_joint_delta(seed_angles, phase_points)
            diagnostics.append({"steps": steps, "max_step_delta_deg": observed_delta})
            if observed_delta <= max_step_delta or steps >= max_steps:
                if diagnostics:
                    for point in phase_points:
                        point.orientation_fallback = bool(point.orientation_fallback)
                return current_seed, None, diagnostics

            next_steps = int(np.ceil(steps * max(1.5, observed_delta / max_step_delta)))
            steps = min(max_steps, max(steps + 1, next_steps))
            if steps >= max_steps and diagnostics[-1]["steps"] >= max_steps:
                break

        del trajectory[start_len:]
        if last_reason:
            return seed_angles, last_reason, diagnostics
        return (
            seed_angles,
            f"{phase_name} 自动加密到 {max_steps} 点后仍然关节跳变过大，"
            f"请调整 ready_pose 或目标点。",
            diagnostics,
        )

    def _local_ik_error_context(
        self,
        trajectory,
        phase_start,
        point_index,
        current_debug,
        next_pose=None,
        seed_angles=None,
        gripper_target=None,
        phase_name=None,
    ):
        """返回失败点前后误差，帮助判断是否局部数值失败。"""
        previous_point = trajectory[-1] if len(trajectory) > phase_start else None
        context = {
            "previous_point_error_mm": previous_point.position_error_mm if previous_point else None,
            "current_point_error_mm": current_debug.get("position_error_mm"),
            "next_point_error_mm": None,
            "note": "",
        }
        if next_pose is not None and seed_angles is not None:
            solver_phase_name = phase_ik_name(phase_name) if phase_name is not None else phase_name
            try:
                _next_angles, next_debug = self.ik_solver.solve(
                    next_pose,
                    seed_angles,
                    self.hit_config,
                    gripper_target,
                    solver_phase_name,
                    max_iter=int(self.controller_config["ik"].get("max_iter", 200)) * 3,
                    force_position_only=(solver_phase_name in {"approach_above_target"}),
                )
                context["next_point_error_mm"] = next_debug.get("position_error_mm")
            except Exception as exc:  # noqa: BLE001 - 这里只做诊断，不影响原始错误。
                context["next_point_error"] = str(exc)
        if point_index <= 1:
            context["note"] = "失败点是该阶段首点，优先检查阶段衔接或目标边界。"
        elif context["next_point_error_mm"] is not None:
            context["note"] = "若前后点误差明显更小，通常是局部 IK 数值失败。"
        return context

    def copy_reversed_phase(self, source_points, phase_name, profile_name=None):
        """deepcopy 已有轨迹的反向路径，只改 phase/speed/acc/dt。

        phase_name 保留“回程正在反向哪个去程阶段”的语义；
        profile_name 只决定速度/加速度/延时，不改变原轨迹数据。
        """
        profile = phase_profile(self.hit_config, profile_name or phase_name)
        reversed_points = copy.deepcopy(list(reversed(source_points)))
        for point in reversed_points:
            point.phase = phase_name
            point.speed = int(profile.speed)
            point.acc = int(profile.acc)
            point.dt = float(profile.dt)
        return reversed_points

    def build_press_motion(
        self,
        trajectory,
        target_name,
        above_pose,
        contact_pose,
        seed_angles,
        gripper_target,
        approach_phase,
        press_phase,
        hold_phase,
        lift_phase,
        contact_dwell_key,
        use_fk_above_for_press=False,
    ):
        """追加一套通用下压动作：above -> contact -> dwell -> above。

        hit 和 reload 都通过这里生成下压轨迹；二者只更换 target_pose_base
        和输出阶段名，避免 reload 走独立姿态/轨迹逻辑。
        """
        action = self.hit_config["hit_action"]
        approach_start = len(trajectory)
        solver_phase_name = phase_ik_name(approach_phase)
        try:
            above_angles, above_debug = self.ik_solver.solve(
                above_pose,
                seed_angles,
                self.hit_config,
                gripper_target,
                solver_phase_name,
                force_position_only=(solver_phase_name in {"approach_above_target"}),
            )
        except Exception as exc:  # noqa: BLE001 - 返回 PlanResult 给 CLI/ROS 处理。
            return seed_angles, None, f"{target_name} above 单点 IK 求解失败：{exc}"

        reason = self.safety_checker.validate_solution(
            approach_phase,
            1,
            above_pose,
            above_angles,
            above_debug,
            max_position_error_mm=phase_position_error_limit_mm(
                self.hit_config,
                self.controller_config,
                approach_phase,
            ),
        )
        if reason:
            return seed_angles, None, reason

        approach_profile = phase_profile(self.hit_config, approach_phase)
        approach_start_pose = self.ik_solver.fk_pose(seed_angles, above_pose)
        trajectory.append(
            self.exact_joint_point(
                approach_phase,
                approach_start_pose,
                seed_angles,
                approach_profile,
            )
        )
        seed, reason = self.append_joint_phase(
            trajectory,
            approach_phase,
            seed_angles,
            above_angles,
            above_pose,
        )
        if reason:
            return seed_angles, None, reason
        approach_points = list(trajectory[approach_start:])
        seed = dict(approach_points[-1].angles) if approach_points else dict(above_angles)

        press_above_pose = above_pose
        press_contact_pose = contact_pose
        if use_fk_above_for_press:
            press_depth = float(above_pose.z) - float(contact_pose.z)
            press_above_pose = self.ik_solver.fk_pose(seed, above_pose)
            press_contact_pose = Pose6D(
                x=float(press_above_pose.x),
                y=float(press_above_pose.y),
                z=float(press_above_pose.z) - press_depth,
                roll=float(press_above_pose.roll),
                pitch=float(press_above_pose.pitch),
                yaw=float(press_above_pose.yaw),
                frame=press_above_pose.frame,
            )

        press_start = len(trajectory)

        def press_poses_factory(steps):
            return interpolate_pose_z(press_above_pose, press_contact_pose, steps)

        seed, reason, phase_adaptive = self.append_adaptive_cartesian_phase(
            trajectory,
            press_phase,
            press_poses_factory,
            seed,
            gripper_target,
        )
        if reason:
            return seed_angles, None, reason
        press_points = list(trajectory[press_start:])

        hold_profile = phase_profile(self.hit_config, hold_phase)
        hold_profile.dt = float(
            action.get(
                contact_dwell_key,
                action.get("dwell_s", hold_profile.dt),
            )
        )
        hold_point = self.exact_joint_point(hold_phase, press_contact_pose, seed, hold_profile)
        trajectory.append(hold_point)
        hold_points = [hold_point]

        lift_points = self.copy_reversed_phase(
            press_points,
            lift_phase,
            profile_name=lift_phase,
        )
        trajectory.extend(lift_points)
        if lift_points:
            seed = dict(lift_points[-1].angles)

        return seed, {
            "target_pose": press_contact_pose,
            "above_pose": press_above_pose,
            "contact_pose": press_contact_pose,
            "approach_points": approach_points,
            "press_points": press_points,
            "hold_points": hold_points,
            "lift_points": lift_points,
            "approach_debug": above_debug,
            "press_adaptive": phase_adaptive,
        }, None

    def plan_hit(self, target_pose_base, current_angles, gripper_target):
        """规划一次完整打靶动作。"""
        home_angles = self.home_angles(current_angles, gripper_target)
        differences = self.safety_checker.home_differences(
            current_angles,
            home_angles,
            self.home_tolerance_deg(),
        )

        poses = {}
        home_pose = self.ik_solver.fk_pose(home_angles, target_pose_base)
        poses["home"] = home_pose
        ready_angles = self.ready_angles(current_angles, gripper_target)
        ready_pose = self.ik_solver.fk_pose(ready_angles, target_pose_base)
        poses["ready"] = ready_pose
        reload_target_pose = self.build_reload_target_pose(target_pose_base)

        trajectory = []
        adaptive_diagnostics = {}
        auto_return_points = []

        if differences:
            auto_return_start = len(trajectory)
            seed, reason = self.append_joint_phase(
                trajectory,
                "auto_return_home",
                current_angles,
                home_angles,
                target_pose_base,
            )
            if reason:
                return PlanResult(
                    False,
                    "自动回 home 失败：\n" + reason,
                    trajectory,
                    poses,
                    diagnostics={"home_differences": differences},
                )
            auto_return_points = list(trajectory[auto_return_start:])
            adaptive_diagnostics["auto_return_home"] = [
                {
                    "mode": "current_to_home_joint_space",
                    "points": len(auto_return_points),
                    "home_delta_count": len(differences),
                }
            ]
        else:
            seed = dict(home_angles)

        home_profile = phase_profile(self.hit_config, "return_home")
        home_point = self.exact_joint_point("home", home_pose, home_angles, home_profile)
        trajectory.append(home_point)
        home_points = [home_point]
        seed = dict(home_angles)

        ready_start = len(trajectory)
        seed, reason = self.append_joint_phase(
            trajectory,
            "move_to_ready",
            home_angles,
            ready_angles,
            ready_pose,
        )
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        move_to_ready_points = list(trajectory[ready_start:])

        reload_motion = None
        hit_motion = None
        reload_points = []
        reload_press_points = []
        reload_hold_points = []
        reload_lift_points = []
        if reload_target_pose is None:
            return PlanResult(False, "reload_pose is disabled or missing.", trajectory, poses)

        reload_contact_pose = self.build_reload_contact_pose(reload_target_pose)
        seed, reload_motion, reason = self.build_press_motion(
            trajectory,
            "reload",
            reload_target_pose,
            reload_contact_pose,
            seed,
            gripper_target,
            approach_phase="move_to_reload",
            press_phase="reload_hit_down",
            hold_phase="reload_hold",
            lift_phase="reload_hit_up",
            contact_dwell_key="hit_contact_dwell_s",
        )
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        poses["reload"] = reload_motion["above_pose"]
        poses["reload_target"] = reload_motion["target_pose"]
        poses["reload_above"] = reload_motion["above_pose"]
        poses["reload_contact"] = reload_motion["contact_pose"]
        reload_points = reload_motion["approach_points"]
        reload_press_points = reload_motion["press_points"]
        reload_hold_points = reload_motion["hold_points"]
        reload_lift_points = reload_motion["lift_points"]
        adaptive_diagnostics["move_to_reload"] = [
            {
                "mode": "shared_press_motion_approach",
                "points": len(reload_points),
                "ik_error_mm": reload_motion["approach_debug"].get("position_error_mm"),
            }
        ]
        adaptive_diagnostics["reload_hit_down"] = reload_motion["press_adaptive"]

        hit_poses = self.build_hit_poses(target_pose_base)
        seed, hit_motion, reason = self.build_press_motion(
            trajectory,
            "hit",
            hit_poses["above_target"],
            hit_poses["contact"],
            seed,
            gripper_target,
            approach_phase="move_to_hit_above",
            press_phase="hit_down",
            hold_phase="hit_hold",
            lift_phase="hit_up",
            contact_dwell_key="hit_contact_dwell_s",
        )
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        poses["hit_above"] = hit_motion["above_pose"]
        poses["hit_contact"] = hit_motion["contact_pose"]
        poses["above_target"] = hit_motion["above_pose"]
        poses["contact"] = hit_motion["contact_pose"]
        approach_points = hit_motion["approach_points"]
        strike_points = hit_motion["press_points"]
        hit_hold_points = hit_motion["hold_points"]
        return_strike_points = hit_motion["lift_points"]
        adaptive_diagnostics["move_to_hit_above"] = [
            {
                "mode": "shared_press_motion_approach",
                "points": len(approach_points),
                "ik_error_mm": hit_motion["approach_debug"].get("position_error_mm"),
            }
        ]
        adaptive_diagnostics["hit_down"] = hit_motion["press_adaptive"]
        strike_contract = assert_strike_cartesian_contract(
            strike_points,
            target_pose_base,
            poses,
            self.hit_config,
            self.controller_config,
        )
        if not strike_contract["ok"]:
            return PlanResult(
                False,
                strike_contract["reason"],
                trajectory,
                poses,
                diagnostics={"strike_contract": strike_contract},
            )

        return_to_ready_start = len(trajectory)
        seed, reason = self.append_joint_phase(
            trajectory,
            "return_to_ready",
            seed,
            ready_angles,
            ready_pose,
        )
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        return_to_ready_points = list(trajectory[return_to_ready_start:])

        return_home_points = self.copy_reversed_phase(
            home_points + move_to_ready_points,
            "return_home",
            profile_name="return_move_to_ready",
        )
        reverse_check = {
            "ok": True,
            "reason": "",
            "reload_hit_down_vs_reload_hit_up": compare_reversed_points(
                reload_press_points,
                reload_lift_points,
            ),
            "hit_down_vs_hit_up": compare_reversed_points(strike_points, return_strike_points),
            "home_ready_vs_return_home": compare_reversed_points(
                home_points + move_to_ready_points,
                return_home_points,
            ),
        }
        reverse_check["reload_press_down_vs_lift_up"] = reverse_check["reload_hit_down_vs_reload_hit_up"]
        reverse_check["strike_down_vs_return_strike_down"] = reverse_check["hit_down_vs_hit_up"]
        reverse_check["ok"] = all(
            bool(item.get("ok", False))
            for key, item in reverse_check.items()
            if isinstance(item, dict) and key not in {"reason"}
        )
        if not reverse_check["ok"]:
            reverse_check["reason"] = "至少一个回程阶段不是对应去程阶段的严格反序。"
        if not reverse_check["ok"]:
            return PlanResult(
                False,
                reverse_check.get("reason", "回程轨迹反向校验失败。"),
                trajectory,
                poses,
                diagnostics={"reverse_check": reverse_check},
            )
        trajectory.extend(return_home_points)

        return PlanResult(
            True,
            trajectory=trajectory,
            poses=poses,
            diagnostics={
                "return_policy": (
                    "路线：必要时 auto_return_home -> home -> ready -> reload_above -> reload_contact "
                    "-> reload_above -> hit_above -> hit_contact -> hit_above -> ready -> home。"
                    "reload 和 hit 下压都调用同一套 press motion 构建逻辑，只替换目标坐标。"
                ),
                "strict_reverse_return": False,
                "route": "auto-home-ready-reload-press-hit-press-ready-home",
                "auto_return_home_used": bool(auto_return_points),
                "home_differences": differences,
                "auto_return_home_points": len(auto_return_points),
                "move_to_ready_points": len(move_to_ready_points),
                "move_to_reload_points": len(reload_points),
                "reload_hit_down_points": len(reload_press_points),
                "reload_hit_up_points": len(reload_lift_points),
                "reload_press_down_points": len(reload_press_points),
                "reload_hold_points": len(reload_hold_points),
                "reload_lift_up_points": len(reload_lift_points),
                "return_reload_to_ready_points": 0,
                "move_to_hit_above_points": len(approach_points),
                "hit_down_points": len(strike_points),
                "hit_up_points": len(return_strike_points),
                "move_to_target_above_points": len(approach_points),
                "target_hit_down_points": len(strike_points),
                "target_hit_up_points": len(return_strike_points),
                "approach_points": len(approach_points),
                "home_points": len(home_points),
                "strike_points": len(strike_points),
                "hit_hold_points": len(hit_hold_points),
                "return_strike_down_points": len(return_strike_points),
                "return_to_ready_points": len(return_to_ready_points),
                "return_to_reload_points": 0,
                "return_approach_above_target_points": len(return_to_ready_points),
                "return_home_points": len(return_home_points),
                "reverse_check": reverse_check,
                "strike_contract": strike_contract,
                "adaptive_cartesian_steps": adaptive_diagnostics,
            },
        )


def plan_return_home(current_angles, home_angles, controller_config, hit_config, speed=None, acc=None, steps=None, dt=None):
    """从任意当前姿态低速平滑回到配置 home。"""
    profile = phase_profile(hit_config, "return_home")
    if speed is not None:
        profile.speed = int(speed)
    if acc is not None:
        profile.acc = int(acc)
    if dt is not None:
        profile.dt = float(dt)
    base_steps = int(steps) if steps is not None else profile.steps
    total_steps = adaptive_steps(base_steps, current_angles, home_angles, hit_config)
    trajectory = []
    dummy_pose = Pose6D(float("nan"), float("nan"), float("nan"), 0.0, 0.0, 0.0, "joint_home")
    for point in interpolate_joint_angles_smooth(current_angles, home_angles, total_steps):
        trajectory.append(
            TrajectoryPoint(
                phase="return_home",
                pose=dummy_pose,
                angles=point,
                raw=target_raw_from_angles(point, controller_config),
                speed=profile.speed,
                acc=profile.acc,
                dt=profile.dt,
            )
        )
    return PlanResult(True, trajectory=trajectory, poses={"home": dummy_pose})
