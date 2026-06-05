"""微雪 STServo 总线执行层封装。

这里不修改底层 SDK 和总线协议，只控制高层轨迹如何下发：
- 默认只发送主动关节，避免 wrist_roll / gripper 被反复写入造成抖动。
- 对 raw 目标设置死区，小于死区的微小变化不重复发送。
- 整帧没有有效变化时跳过发送，降低控制指令频率。
"""

from __future__ import annotations

import time

from core.types import ACTIVE_JOINTS, JointState, TrajectoryPoint
from send_absolute_pose_template import angle_to_raw, open_bus, read_current_joint_angles
from stservo_common import COMM_SUCCESS, check_comm


def target_raw_from_angles(target_angles, controller_config):
    """把角度目标转换成 STS3215 raw 位置。"""
    return {
        name: angle_to_raw(controller_config["_joint_by_name"][name], angle)
        for name, angle in target_angles.items()
    }


class ServoInterface:
    """总线连接、读取和轨迹发送。"""

    def __init__(self, controller_config):
        self.controller_config = controller_config
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        """打开串口。"""
        self.port_handler, self.packet_handler = open_bus(
            self.controller_config["serial"]["port"],
            int(self.controller_config["serial"]["baudrate"]),
        )

    def close(self):
        """关闭串口。"""
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler = None
        self.packet_handler = None

    def read_state(self) -> JointState:
        """读取当前六个关节。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        angles, raw = read_current_joint_angles(self.packet_handler, self.controller_config)
        return JointState(angles=angles, raw=raw)

    def send_point(self, point: TrajectoryPoint, joints=None):
        """发送单帧关节目标。

        joints 为空时只发送主动关节。打靶阶段 wrist_roll 和 gripper 默认保持当前状态。
        """
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        for name in list(joints or ACTIVE_JOINTS):
            joint_config = self.controller_config["_joint_by_name"][name]
            scs_id = int(joint_config["id"])
            raw = int(point.raw[name])
            result, error = self.packet_handler.WritePosEx(scs_id, raw, int(point.speed), int(point.acc))
            check_comm(self.packet_handler, scs_id, result, error, "WritePosEx")

    def send_point_sync(self, point: TrajectoryPoint, joints=None):
        """用 SyncWrite 同步发送单帧目标，减少多个舵机启动时间差。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        joints = list(joints or ACTIVE_JOINTS)
        self.packet_handler.groupSyncWrite.clearParam()
        for name in joints:
            joint_config = self.controller_config["_joint_by_name"][name]
            scs_id = int(joint_config["id"])
            raw = int(point.raw[name])
            if not self.packet_handler.SyncWritePosEx(scs_id, raw, int(point.speed), int(point.acc)):
                raise RuntimeError(f"[ID:{scs_id:03d}] SyncWritePosEx addParam failed")
        result = self.packet_handler.groupSyncWrite.txPacket()
        self.packet_handler.groupSyncWrite.clearParam()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"SyncWritePosEx txPacket failed: {self.packet_handler.getTxRxResult(result)}")

    def read_raw_positions(self):
        """读取六个舵机 raw 位置。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        raw = {}
        for joint_config in self.controller_config["joints"]:
            scs_id = int(joint_config["id"])
            position, _speed, result, error = self.packet_handler.ReadPosSpeed(scs_id)
            check_comm(self.packet_handler, scs_id, result, error, "ReadPosSpeed")
            raw[joint_config["name"]] = int(position)
        return raw

    def wait_until_reached(self, target_raw, tolerance_raw: int, timeout_s: float, poll_interval_s: float, joints=None):
        """等待当前 raw 位置接近目标点。

        只等待实际发送的关节，避免未发送的新微小目标把等待逻辑拖住。
        返回 (reached, max_error, current_raw)。
        """
        deadline = time.monotonic() + float(timeout_s)
        joints = list(joints or ACTIVE_JOINTS)
        last_raw = {}
        last_max_error = 0
        while True:
            last_raw = self.read_raw_positions()
            errors = [abs(int(last_raw[joint]) - int(target_raw[joint])) for joint in joints]
            last_max_error = max(errors) if errors else 0
            if last_max_error <= int(tolerance_raw):
                return True, last_max_error, last_raw
            if time.monotonic() >= deadline:
                return False, last_max_error, last_raw
            time.sleep(float(poll_interval_s))

    def _joints_to_send(self, point: TrajectoryPoint, command_joints, last_sent_raw, raw_deadband):
        """根据 raw 死区判断这一帧哪些关节需要真正下发。"""
        joints = []
        for joint in command_joints:
            previous_raw = last_sent_raw.get(joint)
            target_raw = int(point.raw[joint])
            if previous_raw is None or abs(target_raw - int(previous_raw)) >= int(raw_deadband):
                joints.append(joint)
        return joints

    def execute_trajectory(self, trajectory: list[TrajectoryPoint], action_config: dict):
        """按轨迹逐帧发送。"""
        previous_phase = None
        wait_enabled = bool(action_config.get("strict_servo_wait", False))
        debug_visible_wait = bool(action_config.get("debug_visible_strike", False))
        wait_phases = set(
            action_config.get(
                "strict_servo_wait_phases",
                [
                    "approach_above_target",
                    "strike_down",
                    "return_strike_down",
                    "return_approach_above_target",
                    "return_move_to_ready",
                    "return_home",
                ],
            )
        )
        debug_wait_phases = set(
            action_config.get(
                "debug_visible_wait_phases",
                ["approach_above_target", "strike_down", "return_strike_down"],
            )
        )
        command_joints = list(action_config.get("command_joints", ACTIVE_JOINTS))
        raw_deadband = int(action_config.get("command_raw_deadband", 2))
        skip_unchanged_points = bool(action_config.get("skip_unchanged_points", True))
        send_full_frame = bool(action_config.get("send_full_active_joint_frame", True))
        tolerance_raw = int(action_config.get("wait_raw_tolerance", 25))
        timeout_s = float(action_config.get("wait_timeout_s", 0.8))
        poll_interval_s = float(action_config.get("wait_poll_interval_s", 0.02))
        timeout_policy = str(action_config.get("wait_timeout_policy", "warn")).lower()
        use_sync_write = bool(action_config.get("sync_write", False))
        last_sent_raw = {}

        for point in trajectory:
            if previous_phase == "approach_above_target" and point.phase == "strike_down":
                time.sleep(float(action_config.get("pre_strike_dwell_s", 0.0)))
            if previous_phase == "strike_down" and point.phase == "return_strike_down":
                time.sleep(float(action_config.get("hit_hold_s", action_config.get("dwell_s", 0.0))))
            if previous_phase == "return_strike_down" and point.phase == "return_approach_above_target":
                time.sleep(float(action_config.get("after_rise_dwell_s", 0.0)))

            send_joints = self._joints_to_send(point, command_joints, last_sent_raw, raw_deadband)
            sent_this_point = bool(send_joints)
            if sent_this_point:
                frame_joints = command_joints if send_full_frame else send_joints
                if use_sync_write:
                    self.send_point_sync(point, joints=frame_joints)
                else:
                    self.send_point(point, joints=frame_joints)
                for joint in frame_joints:
                    last_sent_raw[joint] = int(point.raw[joint])
            elif not skip_unchanged_points:
                if use_sync_write:
                    self.send_point_sync(point, joints=command_joints)
                else:
                    self.send_point(point, joints=command_joints)
                for joint in command_joints:
                    last_sent_raw[joint] = int(point.raw[joint])
                sent_this_point = True

            should_wait = (wait_enabled and point.phase in wait_phases) or (
                debug_visible_wait and point.phase in debug_wait_phases
            )
            if should_wait and sent_this_point:
                reached, max_error, current_raw = self.wait_until_reached(
                    last_sent_raw,
                    tolerance_raw=tolerance_raw,
                    timeout_s=timeout_s,
                    poll_interval_s=poll_interval_s,
                    joints=(command_joints if send_full_frame else send_joints) or command_joints,
                )
                if not reached:
                    message = (
                        f"{point.phase} 等待到位超时：max_raw_error={max_error} "
                        f"tolerance={tolerance_raw} target={last_sent_raw} current={current_raw}"
                    )
                    if timeout_policy == "error":
                        raise RuntimeError(message)
                    print(f"警告：{message}；本次按 wait_timeout_policy=warn 继续执行。")

            previous_phase = point.phase
            time.sleep(float(point.dt))
