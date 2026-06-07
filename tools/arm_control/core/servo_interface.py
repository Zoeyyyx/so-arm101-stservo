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

    def reconnect(self):
        """重新打开串口，用于恢复刚启动时的瞬时通信失败。"""
        self.close()
        time.sleep(float(self.controller_config.get("serial", {}).get("reconnect_delay_s", 0.25)))
        self.connect()

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
        attempts = max(1, int(self.controller_config.get("serial", {}).get("startup_read_retries", 2)))
        last_error = None
        for attempt in range(1, attempts + 1):
            try:
                angles, raw = read_current_joint_angles(self.packet_handler, self.controller_config)
                break
            except RuntimeError as exc:
                last_error = exc
                if attempt >= attempts:
                    raise
                print(f"警告：首次读取舵机状态失败，正在重连后重试 ({attempt}/{attempts - 1})：{exc}")
                self.reconnect()
        else:
            raise last_error
        return JointState(angles=angles, raw=raw)

    def joint_motion_profile(self, point: TrajectoryPoint, joint_name: str, action_config: dict | None = None):
        """返回单个关节实际下发的 speed/acc，支持按阶段和关节覆盖。"""
        speed = int(point.speed)
        acc = int(point.acc)
        overrides = (action_config or {}).get("joint_speed_overrides", {})
        phase_overrides = overrides.get(point.phase, {})
        joint_override = phase_overrides.get(joint_name, {})
        if "speed" in joint_override:
            speed = int(joint_override["speed"])
        if "acc" in joint_override:
            acc = int(joint_override["acc"])
        return speed, acc

    def send_point(self, point: TrajectoryPoint, joints=None, action_config=None):
        """发送单帧关节目标。

        joints 为空时只发送主动关节。打靶阶段 wrist_roll 和 gripper 默认保持当前状态。
        """
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        for name in list(joints or ACTIVE_JOINTS):
            joint_config = self.controller_config["_joint_by_name"][name]
            scs_id = int(joint_config["id"])
            raw = int(point.raw[name])
            speed, acc = self.joint_motion_profile(point, name, action_config)
            result, error = self.packet_handler.WritePosEx(scs_id, raw, speed, acc)
            check_comm(self.packet_handler, scs_id, result, error, "WritePosEx")

    def send_point_sync(self, point: TrajectoryPoint, joints=None, action_config=None):
        """用 SyncWrite 同步发送单帧目标，减少多个舵机启动时间差。"""
        if self.packet_handler is None:
            raise RuntimeError("ServoInterface 尚未 connect")
        joints = list(joints or ACTIVE_JOINTS)
        self.packet_handler.groupSyncWrite.clearParam()
        for name in joints:
            joint_config = self.controller_config["_joint_by_name"][name]
            scs_id = int(joint_config["id"])
            raw = int(point.raw[name])
            speed, acc = self.joint_motion_profile(point, name, action_config)
            if not self.packet_handler.SyncWritePosEx(scs_id, raw, speed, acc):
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

    def _raw_error(self, current_raw, target_raw, joints):
        """计算一组关节相对目标 raw 的最大误差。"""
        errors = [abs(int(current_raw[joint]) - int(target_raw[joint])) for joint in joints]
        return max(errors) if errors else 0

    def wait_until_reached(self, target_raw, tolerance_raw: int, timeout_s: float, poll_interval_s: float, joints=None):
        """等待当前 raw 位置接近目标点。

        只等待实际发送的关节，避免未发送的新微小目标把等待逻辑拖住。
        返回 (reached, max_error, current_raw, stats)。
        """
        start_time = time.monotonic()
        deadline = start_time + float(timeout_s)
        joints = list(joints or ACTIVE_JOINTS)
        last_raw = {}
        last_max_error = 0
        error_samples = []
        while True:
            last_raw = self.read_raw_positions()
            last_max_error = self._raw_error(last_raw, target_raw, joints)
            error_samples.append(last_max_error)
            if last_max_error <= int(tolerance_raw):
                elapsed = time.monotonic() - start_time
                return True, last_max_error, last_raw, {
                    "elapsed_time": elapsed,
                    "avg_error": sum(error_samples) / len(error_samples),
                    "max_error": max(error_samples),
                    "samples": len(error_samples),
                }
            if time.monotonic() >= deadline:
                elapsed = time.monotonic() - start_time
                return False, last_max_error, last_raw, {
                    "elapsed_time": elapsed,
                    "avg_error": sum(error_samples) / len(error_samples),
                    "max_error": max(error_samples),
                    "samples": len(error_samples),
                }
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
        focus_joint = str(action_config.get("diagnostic_focus_joint", "shoulder_lift"))
        wait_enabled = bool(action_config.get("strict_servo_wait", False))
        debug_visible_wait = bool(action_config.get("debug_visible_strike", False))
        wait_phases = set(
            action_config.get(
                "strict_servo_wait_phases",
                [
                    "hit1_above",
                    "hit1_down",
                    "hit1_up",
                    "hit2_above",
                    "hit2_down",
                    "hit2_up",
                    "return_ready",
                    "return_home",
                ],
            )
        )
        debug_wait_phases = set(
            action_config.get(
                "debug_visible_wait_phases",
                ["hit1_above", "hit1_down", "hit1_up", "hit2_above", "hit2_down", "hit2_up"],
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
        phase_end_wait_enabled = bool(action_config.get("wait_at_phase_end", True))
        phase_end_wait_phases = set(
            action_config.get(
                "phase_end_wait_phases",
                [
                    "home",
                    "move_to_ready",
                    "auto_return_home",
                    "hit1_above",
                    "hit1_down",
                    "hit1_up",
                    "hit2_above",
                    "hit2_down",
                    "hit2_up",
                    "return_ready",
                    "return_home",
                ],
            )
        )
        phase_entry_probe_phases = set(action_config.get("phase_entry_probe_phases", ["hit1_above"]))
        runtime_detail_phases = set(
            action_config.get("runtime_detail_phases", ["hit1_above", "hit2_above"])
        )
        last_sent_raw = {}
        phase_stats = []
        current_stats = None
        last_phase_end_wait = None

        def start_phase(phase):
            return {
                "phase": phase,
                "start_time": time.monotonic(),
                "duration": 0.0,
                "points": 0,
                "sent_points": 0,
                "skipped_points": 0,
                "wait_count": 0,
                "timeout_count": 0,
                "wait_elapsed": 0.0,
                "error_sum": 0.0,
                "error_samples": 0,
                "max_error": 0,
                "focus_joint": focus_joint,
                "focus_error_sum": 0.0,
                "focus_error_samples": 0,
                "focus_max_error": 0,
                "focus_target_delta_sum": 0,
                "focus_target_delta_max": 0,
                "focus_timeout_count": 0,
                "last_focus_target_raw": None,
            }

        def add_error_sample(stats, error):
            stats["error_sum"] += float(error)
            stats["error_samples"] += 1
            stats["max_error"] = max(int(stats["max_error"]), int(error))

        def add_focus_error_sample(stats, current_raw, target_raw):
            joint = stats["focus_joint"]
            if joint not in current_raw or joint not in target_raw:
                return 0
            error = abs(int(current_raw[joint]) - int(target_raw[joint]))
            stats["focus_error_sum"] += float(error)
            stats["focus_error_samples"] += 1
            stats["focus_max_error"] = max(int(stats["focus_max_error"]), int(error))
            return error

        def add_focus_target_delta(stats, target_raw):
            joint = stats["focus_joint"]
            if joint not in target_raw:
                return
            target_value = int(target_raw[joint])
            previous = stats["last_focus_target_raw"]
            if previous is not None:
                delta = abs(target_value - int(previous))
                stats["focus_target_delta_sum"] += int(delta)
                stats["focus_target_delta_max"] = max(int(stats["focus_target_delta_max"]), int(delta))
            stats["last_focus_target_raw"] = target_value

        def raw_errors(current_raw, target_raw, joints):
            return {
                joint: int(current_raw[joint]) - int(target_raw[joint])
                for joint in joints
                if joint in current_raw and joint in target_raw
            }

        def print_raw_error_block(title, target_raw, current_raw, joints, wait_result=None):
            errors = raw_errors(current_raw, target_raw, joints)
            max_error = max((abs(value) for value in errors.values()), default=0)
            print(title)
            if wait_result is not None:
                print(
                    f"  wait_reached={wait_result.get('reached')} "
                    f"wait_elapsed={float(wait_result.get('elapsed_time', 0.0)):.3f}s "
                    f"wait_samples={wait_result.get('samples')} "
                    f"wait_max_raw_error={wait_result.get('max_error')}"
                )
            print(f"  max_raw_error={max_error} tolerance={tolerance_raw}")
            for joint in joints:
                if joint not in target_raw or joint not in current_raw:
                    continue
                print(
                    f"  {joint:14s} target={int(target_raw[joint]):4d} "
                    f"current={int(current_raw[joint]):4d} "
                    f"error={int(current_raw[joint]) - int(target_raw[joint]):+5d}"
                )

        def print_runtime_point_detail(point, point_index, current_raw, seed_angles, sent_this_point, wait_result):
            errors = raw_errors(current_raw, point.raw, command_joints)
            max_error = max((abs(value) for value in errors.values()), default=0)
            achieved = point.achieved_position_m
            if achieved is None:
                achieved_text = f"x={point.pose.x:.4f} y={point.pose.y:.4f} z={point.pose.z:.4f} (planned_fk)"
            else:
                achieved_text = (
                    f"x={float(achieved[0]):.4f} y={float(achieved[1]):.4f} "
                    f"z={float(achieved[2]):.4f}"
                )
            wait_text = "none"
            if wait_result is not None:
                wait_text = (
                    f"reached={wait_result.get('reached')} "
                    f"elapsed={float(wait_result.get('elapsed_time', 0.0)):.3f}s "
                    f"max_error={wait_result.get('max_error')}"
                )
            print(
                f"执行点诊断 {point.phase} #{point_index:03d}: "
                f"sent={sent_this_point} wait={wait_text} following_max_raw_error={max_error}"
            )
            print(
                f"  target_pose: x={point.pose.x:.4f} y={point.pose.y:.4f} "
                f"z={point.pose.z:.4f} frame={point.pose.frame}"
            )
            print(
                f"  achieved_pose: {achieved_text} "
                f"position_error_mm={float(point.position_error_mm):.3f}"
            )
            print(
                "  seed_angles: "
                + " ".join(f"{joint}={float(seed_angles[joint]):.3f}" for joint in command_joints if joint in seed_angles)
            )
            print(
                "  target_raw: "
                + " ".join(f"{joint}={int(point.raw[joint])}" for joint in command_joints if joint in point.raw)
            )
            print(
                "  current_raw: "
                + " ".join(f"{joint}={int(current_raw[joint])}" for joint in command_joints if joint in current_raw)
            )
            print(
                "  following_error: "
                + " ".join(f"{joint}={int(errors[joint]):+d}" for joint in command_joints if joint in errors)
            )

        def finish_phase(stats):
            stats["duration"] = time.monotonic() - float(stats["start_time"])
            phase_stats.append(stats)

        def print_execution_diagnostics(stats_list):
            if not stats_list:
                return
            print("执行诊断日志:")
            for stats in stats_list:
                avg_error = (
                    float(stats["error_sum"]) / int(stats["error_samples"])
                    if int(stats["error_samples"]) > 0
                    else 0.0
                )
                focus_avg_error = (
                    float(stats["focus_error_sum"]) / int(stats["focus_error_samples"])
                    if int(stats["focus_error_samples"]) > 0
                    else 0.0
                )
                print(
                    f"  {stats['phase']:22s} "
                    f"duration={float(stats['duration']):.3f}s "
                    f"points={int(stats['points'])} sent={int(stats['sent_points'])} "
                    f"skipped={int(stats['skipped_points'])} "
                    f"avg_raw_error={avg_error:.1f} max_raw_error={int(stats['max_error'])} "
                    f"waits={int(stats['wait_count'])} settle_time={float(stats['wait_elapsed']):.3f}s "
                    f"timeouts={int(stats['timeout_count'])}"
                )
                print(
                    f"  {'':22s} {stats['focus_joint']}: "
                    f"avg_error={focus_avg_error:.1f} max_error={int(stats['focus_max_error'])} "
                    f"target_delta_sum={int(stats['focus_target_delta_sum'])} "
                    f"target_delta_max={int(stats['focus_target_delta_max'])} "
                    f"timeouts={int(stats['focus_timeout_count'])}"
                )

        for index, point in enumerate(trajectory):
            if current_stats is None or point.phase != current_stats["phase"]:
                if current_stats is not None:
                    finish_phase(current_stats)
                if index > 0 and point.phase in phase_entry_probe_phases:
                    previous_point = trajectory[index - 1]
                    current_raw = self.read_raw_positions()
                    print_raw_error_block(
                        f"进入 {point.phase} 前实际舵机位置检查，上一阶段={previous_point.phase}",
                        previous_point.raw,
                        current_raw,
                        command_joints,
                        wait_result=last_phase_end_wait,
                    )
                current_stats = start_phase(point.phase)
            current_stats["points"] += 1
            is_phase_end = index == len(trajectory) - 1 or trajectory[index + 1].phase != point.phase
            add_focus_target_delta(current_stats, point.raw)
            seed_angles = trajectory[index - 1].angles if index > 0 else point.angles

            if previous_phase == "approach_above_target" and point.phase == "strike_down":
                time.sleep(float(action_config.get("pre_strike_dwell_s", 0.0)))
            if previous_phase == "strike_down" and point.phase == "return_strike_down":
                time.sleep(
                    float(
                        action_config.get(
                            "hit_contact_dwell_s",
                            action_config.get("hit_hold_s", action_config.get("dwell_s", 0.0)),
                        )
                    )
                )
            if previous_phase == "return_strike_down" and point.phase == "return_approach_above_target":
                time.sleep(float(action_config.get("after_rise_dwell_s", 0.0)))

            send_joints = self._joints_to_send(point, command_joints, last_sent_raw, raw_deadband)
            sent_this_point = bool(send_joints)
            if sent_this_point:
                current_stats["sent_points"] += 1
                frame_joints = command_joints if send_full_frame else send_joints
                if use_sync_write:
                    self.send_point_sync(point, joints=frame_joints, action_config=action_config)
                else:
                    self.send_point(point, joints=frame_joints, action_config=action_config)
                for joint in frame_joints:
                    last_sent_raw[joint] = int(point.raw[joint])
            elif not skip_unchanged_points:
                if use_sync_write:
                    self.send_point_sync(point, joints=command_joints, action_config=action_config)
                else:
                    self.send_point(point, joints=command_joints, action_config=action_config)
                for joint in command_joints:
                    last_sent_raw[joint] = int(point.raw[joint])
                sent_this_point = True
                current_stats["sent_points"] += 1
            if not sent_this_point:
                current_stats["skipped_points"] += 1

            should_wait = (wait_enabled and point.phase in wait_phases) or (
                debug_visible_wait and point.phase in debug_wait_phases
            ) or (
                phase_end_wait_enabled and is_phase_end and point.phase in phase_end_wait_phases
            )
            current_raw_sample = None
            wait_result = None
            if should_wait and sent_this_point:
                wait_joints = (command_joints if send_full_frame else send_joints) or command_joints
                reached, max_error, current_raw, wait_stats = self.wait_until_reached(
                    last_sent_raw,
                    tolerance_raw=tolerance_raw,
                    timeout_s=timeout_s,
                    poll_interval_s=poll_interval_s,
                    joints=wait_joints,
                )
                current_stats["wait_count"] += 1
                current_stats["wait_elapsed"] += float(wait_stats["elapsed_time"])
                current_stats["timeout_count"] += 0 if reached else 1
                add_error_sample(current_stats, wait_stats["avg_error"])
                current_stats["max_error"] = max(int(current_stats["max_error"]), int(wait_stats["max_error"]))
                focus_error = add_focus_error_sample(current_stats, current_raw, last_sent_raw)
                if not reached and focus_error > tolerance_raw:
                    current_stats["focus_timeout_count"] += 1
                wait_result = {
                    "reached": bool(reached),
                    "elapsed_time": float(wait_stats["elapsed_time"]),
                    "samples": int(wait_stats["samples"]),
                    "max_error": int(wait_stats["max_error"]),
                    "final_error": int(max_error),
                }
                current_raw_sample = current_raw
                if not reached:
                    message = (
                        f"{point.phase} 等待到位超时：max_raw_error={max_error} "
                        f"tolerance={tolerance_raw} target={last_sent_raw} current={current_raw}"
                    )
                    if timeout_policy == "error":
                        raise RuntimeError(message)
                    print(f"警告：{message}；本次按 wait_timeout_policy=warn 继续执行。")
                if is_phase_end:
                    last_phase_end_wait = wait_result
            elif is_phase_end:
                current_raw = self.read_raw_positions()
                max_error = self._raw_error(current_raw, point.raw, command_joints)
                add_error_sample(current_stats, max_error)
                add_focus_error_sample(current_stats, current_raw, point.raw)
                current_raw_sample = current_raw
                last_phase_end_wait = {
                    "reached": max_error <= tolerance_raw,
                    "elapsed_time": 0.0,
                    "samples": 1,
                    "max_error": int(max_error),
                    "final_error": int(max_error),
                    "sample_only": True,
                }
            elif point.phase in runtime_detail_phases:
                current_raw_sample = self.read_raw_positions()

            if point.phase in runtime_detail_phases and current_raw_sample is not None:
                print_runtime_point_detail(
                    point,
                    current_stats["points"],
                    current_raw_sample,
                    seed_angles,
                    sent_this_point,
                    wait_result,
                )

            previous_phase = point.phase
            time.sleep(float(point.dt))

        if current_stats is not None:
            finish_phase(current_stats)
        print_execution_diagnostics(phase_stats)
