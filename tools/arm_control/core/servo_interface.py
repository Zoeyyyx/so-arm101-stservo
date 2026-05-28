"""微雪 STServo 总线接口封装。"""

from __future__ import annotations

import time

from core.types import ACTIVE_JOINTS, JointState, TrajectoryPoint
from send_absolute_pose_template import angle_to_raw, open_bus, read_current_joint_angles, send_joint_point
from stservo_common import COMM_SUCCESS, check_comm


def target_raw_from_angles(target_angles, controller_config):
    """把角度目标转换成 STS3215 raw 位置。"""
    return {
        name: angle_to_raw(controller_config["_joint_by_name"][name], angle)
        for name, angle in target_angles.items()
    }


class ServoInterface:
    """总线连接、读取和发送。"""

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

    def send_point(self, point: TrajectoryPoint):
        """发送单帧关节目标。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        send_joint_point(
            self.packet_handler,
            point.angles,
            self.controller_config,
            point.speed,
            point.acc,
        )

    def send_point_sync(self, point: TrajectoryPoint):
        """用 SyncWrite 同步发送单帧目标，减少六个舵机启动时间差。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        self.packet_handler.groupSyncWrite.clearParam()
        for joint_config in self.controller_config["joints"]:
            name = joint_config["name"]
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

    def wait_until_reached(self, point: TrajectoryPoint, tolerance_raw: int, timeout_s: float, poll_interval_s: float):
        """等待当前 raw 位置接近目标点。

        返回 (reached, max_error, current_raw)。如果超时，上层可以选择继续或报错。
        """
        deadline = time.monotonic() + float(timeout_s)
        last_raw = {}
        last_max_error = 0
        while True:
            last_raw = self.read_raw_positions()
            # 打靶阶段只等待主动关节到位；wrist_roll 和 gripper 当前保持不动，不参与等待判定。
            errors = [abs(int(last_raw[joint]) - int(point.raw[joint])) for joint in ACTIVE_JOINTS]
            last_max_error = max(errors)
            if last_max_error <= int(tolerance_raw):
                return True, last_max_error, last_raw
            if time.monotonic() >= deadline:
                return False, last_max_error, last_raw
            time.sleep(float(poll_interval_s))

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
        tolerance_raw = int(action_config.get("wait_raw_tolerance", 25))
        timeout_s = float(action_config.get("wait_timeout_s", 0.8))
        poll_interval_s = float(action_config.get("wait_poll_interval_s", 0.02))
        timeout_policy = str(action_config.get("wait_timeout_policy", "warn")).lower()
        use_sync_write = bool(action_config.get("sync_write", False))
        for point in trajectory:
            if previous_phase == "approach_above_target" and point.phase == "strike_down":
                time.sleep(float(action_config.get("pre_strike_dwell_s", 0.0)))
            if previous_phase == "strike_down" and point.phase == "return_strike_down":
                time.sleep(float(action_config.get("hit_hold_s", action_config.get("dwell_s", 0.0))))
            if previous_phase == "return_strike_down" and point.phase == "return_approach_above_target":
                time.sleep(float(action_config.get("after_rise_dwell_s", 0.0)))
            if use_sync_write:
                self.send_point_sync(point)
            else:
                self.send_point(point)
            should_wait = (wait_enabled and point.phase in wait_phases) or (
                debug_visible_wait and point.phase in debug_wait_phases
            )
            if should_wait:
                reached, max_error, current_raw = self.wait_until_reached(
                    point,
                    tolerance_raw=tolerance_raw,
                    timeout_s=timeout_s,
                    poll_interval_s=poll_interval_s,
                )
                if not reached:
                    message = (
                        f"{point.phase} 等待到位超时：max_raw_error={max_error} "
                        f"tolerance={tolerance_raw} target={point.raw} current={current_raw}"
                    )
                    if timeout_policy == "error":
                        raise RuntimeError(message)
                    print(f"警告：{message}；本次按 wait_timeout_policy=warn 继续执行。")
            previous_phase = point.phase
            time.sleep(float(point.dt))
