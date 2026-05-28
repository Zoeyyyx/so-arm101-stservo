"""安全检查：home、工作空间、关节包络和禁入区。"""

from __future__ import annotations

import numpy as np

from core.types import ACTIVE_JOINTS, Pose6D
from send_absolute_pose_template import check_workspace_safety


def point_in_forbidden_zone(point, zone):
    """判断 base 坐标系下的末端点是否进入禁入区。"""
    if zone is None:
        return False
    x, y, z = [float(value) for value in point]
    zone_type = zone.get("type")
    if zone_type == "cylinder":
        center_x, center_y = zone.get("center", [0.0, 0.0])
        radius = float(zone["radius"])
        z_min = float(zone["z_min"])
        z_max = float(zone["z_max"])
        distance_xy = float(np.hypot(x - float(center_x), y - float(center_y)))
        return distance_xy <= radius and z_min <= z <= z_max
    if zone_type == "aabb":
        return (
            float(zone["x_min"]) <= x <= float(zone["x_max"])
            and float(zone["y_min"]) <= y <= float(zone["y_max"])
            and float(zone["z_min"]) <= z <= float(zone["z_max"])
        )
    raise ValueError(f"未知 forbidden zone 类型: {zone_type}")


class SafetyChecker:
    """集中处理所有安全判断。"""

    def __init__(self, controller_config, safe_limits, workspace_bounds, hit_config=None, forbidden_zone=None):
        self.controller_config = controller_config
        self.safe_limits = safe_limits
        self.workspace_bounds = workspace_bounds
        self.hit_config = hit_config or {}
        self.forbidden_zone = forbidden_zone

    def home_differences(self, current_angles, home_angles, tolerance_deg):
        """返回超出 home 容差的关节差异。"""
        differences = []
        for joint in ACTIVE_JOINTS:
            current = float(current_angles[joint])
            home = float(home_angles[joint])
            delta = current - home
            if abs(delta) > float(tolerance_deg):
                differences.append(
                    {
                        "joint": joint,
                        "current": current,
                        "home": home,
                        "delta": delta,
                        "tolerance": float(tolerance_deg),
                    }
                )
        return differences

    def format_home_mismatch(self, differences):
        """把 home 差异整理成可打印文本。"""
        lines = ["当前机械臂姿态不在配置 home/stow 附近。请先运行 return_home.py。"]
        for item in differences:
            lines.append(
                f"  {item['joint']}: current={item['current']:.3f} deg, "
                f"config_home={item['home']:.3f} deg, delta={item['delta']:+.3f} deg "
                f"(tolerance={item['tolerance']:.3f})"
            )
        return "\n".join(lines)

    def validate_pose_workspace(self, pose: Pose6D):
        """检查末端目标是否落在粗略工作空间内。"""
        return check_workspace_safety(pose, self.workspace_bounds, self.controller_config)

    def validate_target_angles(self, angles):
        """只检查主动参与打靶的关节是否在 95% 安全包络内。"""
        violations = []
        for joint in ACTIVE_JOINTS:
            low, high = self.safe_limits[joint]
            value = float(angles[joint])
            if value < low or value > high:
                violations.append((joint, value, low, high))
        return violations

    def validate_solution(self, phase_name, point_index, pose, angles, debug, max_position_error_mm=None):
        """检查单个 IK 点，返回 None 表示通过，否则返回原因字符串。"""
        workspace_violations = self.validate_pose_workspace(pose)
        if workspace_violations:
            detail = "\n".join(f"  {item}" for item in workspace_violations)
            return f"{phase_name} 第 {point_index} 点 workspace 检查失败\n{detail}"

        joint_violations = self.validate_target_angles(angles)
        if joint_violations:
            detail = "\n".join(
                f"  {joint}: target={value:.3f} deg, safe={low:.3f}..{high:.3f} deg"
                for joint, value, low, high in joint_violations
            )
            return f"{phase_name} 第 {point_index} 点关节目标不在 95% 安全包络内\n{detail}"

        max_position_error = float(
            max_position_error_mm
            if max_position_error_mm is not None
            else self.controller_config["workspace"].get("position_error_max_mm", 10.0)
        )
        if float(debug["position_error_mm"]) > max_position_error:
            achieved = debug.get("achieved_position", [float("nan")] * 3)
            return (
                f"{phase_name} 第 {point_index} 点 IK 位置误差过大："
                f"{debug['position_error_mm']:.3f} mm > {max_position_error:.3f} mm\n"
                f"  target: x={pose.x:.4f} y={pose.y:.4f} z={pose.z:.4f}\n"
                f"  achieved: x={achieved[0]:.4f} y={achieved[1]:.4f} z={achieved[2]:.4f}"
            )

        tool_config = self.hit_config.get("tool_orientation", {})
        hard_reject = bool(tool_config.get("hard_reject_orientation_error", False))
        orientation_error = debug.get("orientation_error_deg")
        if hard_reject and orientation_error is not None and debug.get("orientation_requested"):
            max_orientation_error = float(tool_config.get("max_orientation_error_deg", 35.0))
            if orientation_error > max_orientation_error:
                return (
                    f"{phase_name} 第 {point_index} 点工具向下姿态误差过大："
                    f"{orientation_error:.3f} deg > {max_orientation_error:.3f} deg"
                )

        achieved = debug.get("achieved_position")
        if achieved is not None and point_in_forbidden_zone(achieved, self.forbidden_zone):
            return (
                f"{phase_name} 第 {point_index} 点进入 forbidden zone\n"
                f"  achieved: x={achieved[0]:.4f} y={achieved[1]:.4f} z={achieved[2]:.4f}"
            )
        return None
