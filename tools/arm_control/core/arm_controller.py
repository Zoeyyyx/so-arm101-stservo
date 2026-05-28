"""SO101 高层控制器，供 CLI 和未来 ROS 节点复用。"""

from __future__ import annotations

from core.config_loader import (
    load_controller_config,
    load_forbidden_zone,
    load_hit_config,
    load_home_pose,
    load_ready_pose,
    scale_phase_profiles,
    transform_target_to_base,
)
from core.ik_solver import IKSolver
from core.safety_checker import SafetyChecker
from core.servo_interface import ServoInterface
from core.trajectory_planner import HitTrajectoryPlanner, plan_return_home
from core.types import JointState, Pose6D
from send_absolute_pose_template import compute_workspace_bounds, joint_safe_limits


class ArmController:
    """机械臂控制入口。"""

    def __init__(self, hit_config, controller_config, home_pose, ready_pose, forbidden_zone=None):
        self.hit_config = hit_config
        self.controller_config = controller_config
        self.home_pose = home_pose
        self.ready_pose = ready_pose
        self.forbidden_zone = forbidden_zone
        self.safe_limits = joint_safe_limits(controller_config)
        self.workspace_bounds = compute_workspace_bounds(controller_config, self.safe_limits)
        self.ik_solver = IKSolver(controller_config, self.safe_limits)
        self.safety_checker = SafetyChecker(
            controller_config,
            self.safe_limits,
            self.workspace_bounds,
            hit_config=hit_config,
            forbidden_zone=forbidden_zone,
        )
        self.planner = HitTrajectoryPlanner(
            hit_config,
            controller_config,
            home_pose,
            ready_pose,
            self.ik_solver,
            self.safety_checker,
        )
        self.servo = ServoInterface(controller_config)

    @classmethod
    def from_files(
        cls,
        hit_config_path,
        home_config_path=None,
        ready_config_path=None,
        forbidden_zone_path=None,
        port=None,
        baudrate=None,
        workspace_samples=None,
        speed_scale=1.0,
        acc_scale=1.0,
        home_tolerance=None,
    ):
        """从配置文件构造控制器。"""
        hit_config = load_hit_config(hit_config_path)
        scale_phase_profiles(hit_config, speed_scale, acc_scale)
        home_pose = load_home_pose(hit_config, home_config_path)
        ready_pose = load_ready_pose(hit_config, ready_config_path, home_pose=home_pose)
        if home_tolerance is not None:
            home_pose["start_tolerance_deg"] = float(home_tolerance)
        controller_config = load_controller_config(hit_config)
        if port:
            controller_config["serial"]["port"] = port
        if baudrate:
            controller_config["serial"]["baudrate"] = int(baudrate)
        if workspace_samples is not None:
            controller_config["workspace"]["sample_count_per_joint"] = int(workspace_samples)
        forbidden_zone = load_forbidden_zone(forbidden_zone_path) if forbidden_zone_path else None
        return cls(hit_config, controller_config, home_pose, ready_pose, forbidden_zone)

    def connect(self):
        """连接舵机总线。"""
        self.servo.connect()

    def close(self):
        """关闭舵机总线。"""
        self.servo.close()

    def read_state(self) -> JointState:
        """读取当前机械臂状态。"""
        return self.servo.read_state()

    def target_to_base(self, target_pose: Pose6D) -> Pose6D:
        """把输入目标转换到机械臂基座坐标。"""
        return transform_target_to_base(target_pose, self.hit_config)

    def default_gripper(self, state: JointState, override=None):
        """打靶程序不主动控制夹爪，始终保持当前角度。"""
        return float(state.angles["gripper"])

    def plan_hit(self, target_pose: Pose6D, state: JointState, gripper_override=None):
        """规划打靶动作，不执行。"""
        target_pose_base = self.target_to_base(target_pose)
        gripper_target = self.default_gripper(state, gripper_override)
        return self.planner.plan_hit(target_pose_base, state.angles, gripper_target), target_pose_base

    def plan_return_home(self, state: JointState, speed=None, acc=None, steps=None, dt=None):
        """规划从当前位置回 home 的关节空间轨迹。"""
        home_angles = self.planner.home_angles(
            state.angles,
            self.hit_config["hit_action"].get("default_gripper", state.angles["gripper"]),
        )
        return plan_return_home(
            state.angles,
            home_angles,
            self.controller_config,
            self.hit_config,
            speed=speed,
            acc=acc,
            steps=steps,
            dt=dt,
        )

    def execute_trajectory(self, trajectory):
        """执行一条已经规划好的轨迹。"""
        self.servo.execute_trajectory(trajectory, self.hit_config["hit_action"])
