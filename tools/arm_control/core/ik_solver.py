"""IK 和 FK 求解器。

IK chain、URDF 限位等只在对象创建时加载一次，避免每个轨迹点重复 make_chain。
"""

from __future__ import annotations

import numpy as np

from core.types import IK_JOINTS, Pose6D
from send_absolute_pose_template import (
    chain_vector_to_joint_degrees,
    clamp_initial_angles_to_limits,
    get_urdf_path,
    joint_degrees_to_chain_vector,
    make_chain,
    read_urdf_joint_limits_degrees,
)


def normalize_vector(values):
    """单位化向量。"""
    vector = np.array(values, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise ValueError("姿态目标轴不能是零向量")
    return vector / norm


def orientation_error_deg(fk_matrix, mode, target_axis):
    """计算工具轴姿态误差，单位度。"""
    if mode == "X":
        actual = fk_matrix[:3, 0]
    elif mode == "Y":
        actual = fk_matrix[:3, 1]
    elif mode == "Z":
        actual = fk_matrix[:3, 2]
    else:
        return None
    actual = normalize_vector(actual)
    target = normalize_vector(target_axis)
    dot = float(np.clip(np.dot(actual, target), -1.0, 1.0))
    return float(np.rad2deg(np.arccos(dot)))


class IKSolver:
    """SO101 IK/FK 求解器。"""

    def __init__(self, controller_config: dict, safe_limits: dict[str, tuple[float, float]]):
        self.controller_config = controller_config
        self.safe_limits = safe_limits
        self.urdf_path = get_urdf_path(controller_config)
        self.urdf_limits = read_urdf_joint_limits_degrees(self.urdf_path)
        self.ik_limits = self._build_ik_limits()
        self.use_wrist_roll = bool(controller_config["ik"].get("use_wrist_roll", False))
        self.chain = make_chain(self.urdf_path, self.use_wrist_roll, safe_limits=self.ik_limits)

    def _build_ik_limits(self):
        """把配置安全范围和 URDF 范围合并成 IK 可用范围。"""
        limits = {}
        for joint in IK_JOINTS:
            urdf_low, urdf_high = self.urdf_limits.get(joint, self.safe_limits[joint])
            limits[joint] = (
                max(float(self.safe_limits[joint][0]), float(urdf_low)),
                min(float(self.safe_limits[joint][1]), float(urdf_high)),
            )
        return limits

    def fk_position(self, joint_angles: dict[str, float]):
        """计算末端位置，单位米。"""
        chain_vector = joint_degrees_to_chain_vector(joint_angles)
        return self.chain.forward_kinematics(chain_vector)[:3, 3]

    def fk_pose(self, joint_angles: dict[str, float], template_pose: Pose6D) -> Pose6D:
        """用当前关节角计算末端位置，姿态字段沿用模板。"""
        xyz = self.fk_position(joint_angles)
        return Pose6D(
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]),
            template_pose.roll,
            template_pose.pitch,
            template_pose.yaw,
            template_pose.frame,
        )

    def solve(
        self,
        pose: Pose6D,
        seed_angles: dict[str, float],
        hit_config: dict,
        gripper_target,
        phase_name: str,
        max_iter=None,
        force_position_only=False,
    ):
        """对一个末端位姿求 IK。"""
        safe_seed_angles, _ = clamp_initial_angles_to_limits(
            seed_angles,
            self.ik_limits,
            float(self.controller_config["ik"].get("limit_margin_deg", 0.5)),
        )
        initial_chain = joint_degrees_to_chain_vector(safe_seed_angles)
        target_position = np.array([pose.x, pose.y, pose.z], dtype=float)

        tool_config = hit_config["tool_orientation"]
        enforce_orientation = bool(tool_config.get("enforce_tool_down", True))
        tool_down_phases = set(tool_config.get("tool_down_phases", ["strike_down"]))
        orientation_requested = (not force_position_only) and enforce_orientation and phase_name in tool_down_phases
        orientation_mode = tool_config.get("orientation_mode", "Z")
        target_axis = normalize_vector(tool_config.get("target_axis_in_base", [0.0, 0.0, -1.0]))
        iteration_limit = int(max_iter if max_iter is not None else self.controller_config["ik"].get("max_iter", 200))

        def run_ik(request_orientation: bool):
            solution = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_axis if request_orientation else None,
                orientation_mode=orientation_mode if request_orientation else None,
                initial_position=initial_chain,
                max_iter=iteration_limit,
            )
            fk_matrix = self.chain.forward_kinematics(solution)
            achieved = fk_matrix[:3, 3]
            position_error_mm = float(np.linalg.norm(achieved - target_position) * 1000.0)
            orient_error = orientation_error_deg(fk_matrix, orientation_mode, target_axis)
            return solution, achieved, position_error_mm, orient_error

        position_solution, position_achieved, position_error_mm, position_orient_error = run_ik(False)
        solution_chain = position_solution
        achieved_position = position_achieved
        orient_error = position_orient_error
        orientation_fallback = False

        if orientation_requested:
            oriented_solution, oriented_achieved, oriented_pos_error, oriented_orient_error = run_ik(True)
            max_position_error = float(self.controller_config["workspace"].get("position_error_max_mm", 10.0))
            fallback_ratio = float(tool_config.get("fallback_position_error_ratio", 2.0))
            allow_fallback = bool(tool_config.get("position_first_fallback", True))
            fallback_needed = (
                oriented_pos_error > max_position_error
                or oriented_pos_error > max(position_error_mm * fallback_ratio, position_error_mm + 1e-6)
            )
            if allow_fallback and fallback_needed:
                orientation_fallback = True
            else:
                solution_chain = oriented_solution
                achieved_position = oriented_achieved
                position_error_mm = oriented_pos_error
                orient_error = oriented_orient_error

        target_angles = chain_vector_to_joint_degrees(solution_chain)
        # 打靶阶段不使用 wrist_roll 和 gripper 做末端移动。
        # IK 可能会给 wrist_roll 算出一个姿态补偿角，这里强制保持当前 seed，避免它被带动或触发限位。
        target_angles["wrist_roll"] = float(seed_angles["wrist_roll"])
        target_angles["gripper"] = float(seed_angles["gripper"])

        debug = {
            "position_error_mm": position_error_mm,
            "orientation_error_deg": orient_error,
            "orientation_requested": orientation_requested,
            "orientation_fallback": orientation_fallback,
            "achieved_position": achieved_position.tolist(),
            "max_iter": iteration_limit,
            "force_position_only": bool(force_position_only),
        }
        return target_angles, debug
