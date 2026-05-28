"""打靶与回 home 轨迹规划。"""

from __future__ import annotations

import copy
import numpy as np

from core.servo_interface import target_raw_from_angles
from core.types import ACTIVE_JOINTS, ALL_JOINTS, PASSIVE_JOINTS, MotionProfile, PlanResult, Pose6D, TrajectoryPoint


def phase_profile(hit_config, phase_name) -> MotionProfile:
    """读取一个阶段的运动参数。"""
    profile = hit_config["hit_action"]["phases"][phase_name]
    return MotionProfile(
        steps=int(profile["steps"]),
        speed=int(profile["speed"]),
        acc=int(profile["acc"]),
        dt=float(profile["dt"]),
    )


def phase_position_error_limit_mm(hit_config, controller_config, phase_name):
    """按阶段读取允许 IK 位置误差，单位 mm。"""
    action = hit_config.get("hit_action", {})
    default_limit = float(controller_config["workspace"].get("position_error_max_mm", 10.0))
    if phase_name == "approach_above_target":
        return float(action.get("approach_error_max_mm", default_limit))
    if phase_name == "strike_down":
        return float(action.get("strike_error_max_mm", default_limit))
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
        current_seed = dict(start_angles)
        for point_index, angles in enumerate(interpolate_joint_angles_smooth(start_angles, target_angles, steps), start=1):
            joint_violations = self.safety_checker.validate_target_angles(angles)
            if joint_violations:
                detail = "\n".join(
                    f"  {joint}: target={value:.3f} deg, safe={low:.3f}..{high:.3f} deg"
                    for joint, value, low, high in joint_violations
                )
                return current_seed, f"{phase_name} 第 {point_index} 个关节插值点越界：\n{detail}"
            point_pose = self.ik_solver.fk_pose(angles, pose_template)
            trajectory.append(self.exact_joint_point(phase_name, point_pose, angles, profile))
            current_seed = angles
        return current_seed, None

    def append_cartesian_phase(self, trajectory, phase_name, poses, seed_angles, gripper_target):
        """逐点 IK 追加笛卡尔阶段。"""
        profile = phase_profile(self.hit_config, phase_name)
        current_seed = dict(seed_angles)
        phase_start = len(trajectory)
        max_error_mm = phase_position_error_limit_mm(self.hit_config, self.controller_config, phase_name)
        default_max_iter = int(self.controller_config["ik"].get("max_iter", 200))
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
                        phase_name,
                        max_iter=attempt_iter,
                        force_position_only=(phase_name == "approach_above_target"),
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
            trajectory.append(self.make_point(phase_name, pose, angles, debug, profile))
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
            try:
                _next_angles, next_debug = self.ik_solver.solve(
                    next_pose,
                    seed_angles,
                    self.hit_config,
                    gripper_target,
                    phase_name,
                    max_iter=int(self.controller_config["ik"].get("max_iter", 200)) * 3,
                    force_position_only=(phase_name == "approach_above_target"),
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

    def plan_hit(self, target_pose_base, current_angles, gripper_target):
        """规划一次完整打靶动作。"""
        home_angles = self.home_angles(current_angles, gripper_target)
        differences = self.safety_checker.home_differences(
            current_angles,
            home_angles,
            self.home_tolerance_deg(),
        )
        if differences:
            return PlanResult(
                success=False,
                reason=self.safety_checker.format_home_mismatch(differences),
                diagnostics={"home_differences": differences},
            )

        poses = self.build_hit_poses(target_pose_base)
        home_pose = self.ik_solver.fk_pose(home_angles, target_pose_base)
        poses["home"] = home_pose
        ready_angles = self.ready_angles(current_angles, gripper_target)
        ready_pose = self.ik_solver.fk_pose(ready_angles, target_pose_base)
        poses["ready"] = ready_pose

        trajectory = []
        adaptive_diagnostics = {}
        home_profile = phase_profile(self.hit_config, "return_home")
        home_point = self.exact_joint_point("home", home_pose, home_angles, home_profile)
        trajectory.append(home_point)

        ready_start = len(trajectory)
        seed, reason = self.append_joint_phase(
            trajectory,
            "move_to_ready",
            home_angles,
            ready_angles,
            target_pose_base,
        )
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        move_to_ready_points = list(trajectory[ready_start:])

        approach_start = len(trajectory)
        def approach_poses_factory(steps):
            if int(steps) <= 1:
                return [poses["above_target"]]
            return interpolate_pose_xyz(ready_pose, poses["above_target"], steps)

        seed, reason, phase_adaptive = self.append_adaptive_cartesian_phase(
            trajectory,
            "approach_above_target",
            approach_poses_factory,
            seed,
            gripper_target,
        )
        adaptive_diagnostics["approach_above_target"] = phase_adaptive
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        approach_points = list(trajectory[approach_start:])
        home_points = [home_point]
        outbound_points = home_points + move_to_ready_points + approach_points

        strike_start = len(trajectory)
        def strike_poses_factory(steps):
            return interpolate_pose_z(poses["above_target"], poses["contact"], steps)

        seed, reason, phase_adaptive = self.append_adaptive_cartesian_phase(
            trajectory,
            "strike_down",
            strike_poses_factory,
            seed,
            gripper_target,
        )
        adaptive_diagnostics["strike_down"] = phase_adaptive
        if reason:
            return PlanResult(False, reason, trajectory, poses)
        strike_points = list(trajectory[strike_start:])
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

        return_strike_points = self.copy_reversed_phase(
            strike_points,
            "return_strike_down",
            profile_name="return_strike_down",
        )
        return_approach_points = self.copy_reversed_phase(
            approach_points,
            "return_approach_above_target",
            profile_name="return_approach_above_target",
        )
        return_ready_points = self.copy_reversed_phase(
            move_to_ready_points,
            "return_move_to_ready",
            profile_name="return_move_to_ready",
        )
        return_home_points = self.copy_reversed_phase(
            home_points,
            "return_home",
            profile_name="return_home",
        )
        return_points = return_approach_points + return_ready_points + return_home_points
        reverse_check = verify_strict_reverse_return(
            outbound_points,
            strike_points,
            return_strike_points,
            return_points,
        )
        if not reverse_check["ok"]:
            return PlanResult(
                False,
                reverse_check["reason"],
                trajectory,
                poses,
                diagnostics={"reverse_check": reverse_check},
            )
        trajectory.extend(return_strike_points)
        trajectory.extend(return_approach_points)
        trajectory.extend(return_ready_points)
        trajectory.extend(return_home_points)

        return PlanResult(
            True,
            trajectory=trajectory,
            poses=poses,
            diagnostics={
                "return_policy": (
                    "回程按阶段栈反向："
                    "return_strike_down = strike_down 反向；"
                    "return_approach_above_target = approach_above_target 反向；"
                    "return_move_to_ready = move_to_ready 反向；"
                    "return_home = home 反向。"
                ),
                "strict_reverse_return": True,
                "outbound_points": len(outbound_points),
                "move_to_ready_points": len(move_to_ready_points),
                "approach_points": len(approach_points),
                "home_points": len(home_points),
                "strike_points": len(strike_points),
                "return_strike_down_points": len(return_strike_points),
                "return_approach_above_target_points": len(return_approach_points),
                "return_move_to_ready_points": len(return_ready_points),
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
