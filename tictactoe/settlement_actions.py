"""井字棋对局结算动作。

本模块只复用现有 ArmController / ServoInterface，不重写底层串口或 SDK。
动作只控制结算需要的关节，默认预览；只有上层传入 execute=True 才真实下发。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

from .arm_player import TictactoeArmConfig, add_arm_control_paths


DEFAULT_ACTION_CONFIG = Path(__file__).resolve().parents[1] / "config" / "tictactoe_settlement.json"


@dataclass(frozen=True)
class SettlementStep:
    """一个结算动作关键点。"""

    label: str
    joints: dict[str, float]
    duration_s: float


def load_settlement_config(path: str | Path = DEFAULT_ACTION_CONFIG) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as file:
        return json.load(file)


def action_name_for_outcome(outcome: str) -> str | None:
    """把对局结果映射到动作名。"""

    mapping = {
        "robot_win": "victory",
        "draw": "draw_handshake",
        "human_win": "defeat_nod",
    }
    return mapping.get(outcome)


class SettlementActionPlayer:
    """执行机械臂结算动作。"""

    def __init__(
        self,
        arm_config: TictactoeArmConfig,
        *,
        settlement_config_path: str | Path = DEFAULT_ACTION_CONFIG,
        repo_root: str | Path | None = None,
    ):
        self.arm_config = arm_config
        self.settlement_config_path = Path(settlement_config_path)
        self.repo_root = Path(repo_root or Path(__file__).resolve().parents[1])
        self.config = load_settlement_config(self.settlement_config_path)

    def preview_lines(self, outcome: str) -> list[str]:
        action_name = action_name_for_outcome(outcome)
        if action_name is None:
            return [f"结算动作: 无匹配 outcome={outcome}"]
        action = self.config["actions"].get(action_name, {})
        lines = [
            f"结算动作: outcome={outcome} action={action_name}",
            f"  action_duration={float(action.get('duration_s', 3.0)):.2f}s execute=false",
            f"  speed={int(action.get('speed', self.config.get('default_speed', 160)))} "
            f"acc={int(action.get('acc', self.config.get('default_acc', 12)))}",
        ]
        if float(action.get("home_before_s", 0.0)) > 0 or float(action.get("home_after_s", 0.0)) > 0:
            lines.append(
                f"  home_before={float(action.get('home_before_s', 0.0)):.2f}s "
                f"home_after={float(action.get('home_after_s', 0.0)):.2f}s"
            )
        if action_name == "victory":
            total_with_home = (
                float(action.get("home_before_s", 0.0))
                + float(action.get("duration_s", 5.0))
                + float(action.get("home_after_s", 0.0))
            )
            lines.append(
                f"  victory: center_delta={float(action.get('wrist_roll_center_delta_deg', 0.0)):.1f}deg "
                f"swing={float(action.get('wrist_roll_swing_deg', 15.0)):.1f}deg "
                f"gripper_open={float(action.get('gripper_open_deg', 35.0)):.1f}deg "
                f"pulses={int(action.get('pulses', 8))} "
                f"cycles={int(action.get('pulses', 8)) / 2.0:.1f}"
            )
            lines.append(f"  total_with_home={total_with_home:.2f}s")
        if action_name == "draw_handshake":
            total_with_home = (
                float(action.get("home_before_s", 0.0))
                + float(action.get("duration_s", 5.0))
                + float(action.get("home_after_s", 0.0))
            )
            lines.append(
                f"  handshake: center_delta={float(action.get('wrist_flex_center_delta_deg', 90.0)):.1f}deg "
                f"shake_delta={float(action.get('wrist_flex_delta_deg', 8.0)):.1f}deg "
                f"swings={int(action.get('swings', 6))} "
                f"cycles={int(action.get('swings', 6)) / 2.0:.1f} "
                f"center_move={float(action.get('center_move_s', 0.0)):.2f}s"
            )
            lines.append(f"  total_with_home={total_with_home:.2f}s")
        if action_name == "defeat_nod":
            total_with_home = (
                float(action.get("home_before_s", 0.0))
                + float(action.get("wrist_flex_move_s", 0.0))
                + float(action.get("duration_s", 5.0))
                + float(action.get("home_after_s", 0.0))
            )
            lines.append(
                f"  defeat_wrist_flex: delta={float(action.get('wrist_flex_delta_deg', 135.0)):.1f}deg "
                f"move={float(action.get('wrist_flex_move_s', 0.0)):.2f}s "
                f"hold={float(action.get('duration_s', 5.0)):.2f}s"
            )
            lines.append(f"  total_with_home={total_with_home:.2f}s")
        return lines

    def play(self, outcome: str, *, execute: bool = False, port: str | None = None) -> None:
        """预览或执行结算动作。"""

        for line in self.preview_lines(outcome):
            print(line)
        if not execute:
            return

        action_name = action_name_for_outcome(outcome)
        if action_name is None:
            return

        add_arm_control_paths(self.repo_root)
        from core.arm_controller import ArmController  # type: ignore

        controller = ArmController.from_files(
            self.arm_config.hit_config,
            home_config_path=self.arm_config.home_config,
            ready_config_path=self.arm_config.ready_config,
            port=port or self.arm_config.port,
            baudrate=self.arm_config.baudrate,
        )
        controller.connect()
        try:
            state = controller.read_state()
            action = self.config["actions"][action_name]
            speed = int(action.get("speed", self.config.get("default_speed", 160)))
            acc = int(action.get("acc", self.config.get("default_acc", 12)))
            home_speed = int(action.get("home_speed", speed))
            home_acc = int(action.get("home_acc", acc))

            home_before_s = float(action.get("home_before_s", 0.0))
            home_after_s = float(action.get("home_after_s", 0.0))
            if home_before_s > 0.0:
                self.execute_home_transition(controller, state.angles, home_before_s, home_speed, home_acc, "settlement_home_before", action_name)
                state = controller.read_state()

            seed_angles = configured_home_angles(controller, state.angles) if home_before_s > 0.0 else state.angles
            steps = self.build_steps(action_name, seed_angles, controller.controller_config)
            for step in steps:
                raw = {
                    name: angle_to_raw_direct(controller.controller_config, name, angle)
                    for name, angle in step.joints.items()
                }
                point = make_direct_point(step.label, step.joints, raw, speed, acc, step.duration_s)
                print(
                    f"执行结算动作点 {step.label}: "
                    + " ".join(f"{name}={angle:.2f}" for name, angle in step.joints.items())
                )
                if step.label == "defeat_wrist_flex_hold":
                    print(f"  保持 wrist_flex 姿势 {float(step.duration_s):.2f}s")
                controller.servo.send_point(point, joints=list(step.joints), action_config={})
                time.sleep(float(step.duration_s))

            if home_after_s > 0.0:
                state = controller.read_state()
                self.execute_home_transition(controller, state.angles, home_after_s, home_speed, home_acc, "settlement_home_after", action_name)
        finally:
            controller.close()

    def execute_home_transition(
        self,
        controller,
        current_angles: dict[str, float],
        duration_s: float,
        speed: int,
        acc: int,
        label: str,
        action_name: str,
    ) -> None:
        home_angles = configured_home_angles(controller, current_angles)
        steps = max(2, int(self.config.get("home_steps", 12)))
        action_steps = int(
            self.config.get("actions", {})
            .get(action_name, {})
            .get("home_steps", steps)
        )
        steps = max(2, action_steps)
        dt = float(duration_s) / steps
        print(f"执行 {label}: duration={duration_s:.2f}s steps={steps} speed={speed} acc={acc}")
        for index, angles in enumerate(interpolate_joint_angles(current_angles, home_angles, steps), start=1):
            raw = {
                name: angle_to_raw_direct(controller.controller_config, name, angle)
                for name, angle in angles.items()
            }
            point = make_direct_point(f"{label}_{index}", angles, raw, speed, acc, dt)
            controller.servo.send_point(point, joints=list(angles), action_config={})
            time.sleep(dt)

    def build_steps(self, action_name: str, current_angles: dict[str, float], controller_config: dict) -> list[SettlementStep]:
        action = self.config["actions"][action_name]
        duration_s = float(action.get("duration_s", 3.0))

        if action_name == "victory":
            initial_roll_s = max(0.0, float(action.get("initial_roll_s", 0.0)))
            flourish_s = max(0.2, duration_s - initial_roll_s)
            pulses = max(2, int(action.get("pulses", 8)))
            gripper_open = clamp_joint_angle(controller_config, "gripper", float(action.get("gripper_open_deg", 35.0)))
            gripper_close = clamp_joint_angle(controller_config, "gripper", float(action.get("gripper_close_deg", 2.0)))
            roll_center = float(current_angles["wrist_roll"]) + float(action.get("wrist_roll_center_delta_deg", 0.0))
            roll_swing = abs(float(action.get("wrist_roll_swing_deg", 15.0)))
            left = roll_center - roll_swing
            right = roll_center + roll_swing
            steps = []
            if initial_roll_s > 0.0:
                initial_roll = float(current_angles["wrist_roll"]) + float(
                    action.get("wrist_roll_initial_delta_deg", action.get("wrist_roll_center_delta_deg", 0.0))
                )
                steps.append(
                    SettlementStep(
                        label="victory_initial_roll",
                        joints={"gripper": gripper_open, "wrist_roll": initial_roll},
                        duration_s=initial_roll_s,
                    )
                )
            for index in range(pulses):
                steps.append(
                    SettlementStep(
                        label=f"victory_pulse_{index + 1}",
                        joints={
                            "gripper": gripper_open if index % 2 == 0 else gripper_close,
                            "wrist_roll": left if index % 2 == 0 else right,
                        },
                        duration_s=flourish_s / pulses,
                    )
                )
            return steps

        if action_name == "draw_handshake":
            count = max(2, int(action.get("swings", 6)))
            delta = float(action.get("wrist_flex_delta_deg", 8.0))
            center = float(current_angles["wrist_flex"]) + float(action.get("wrist_flex_center_delta_deg", 90.0))
            steps = []
            center_move_s = max(0.0, float(action.get("center_move_s", 0.0)))
            if center_move_s > 0.0:
                steps.append(
                    SettlementStep(
                        label="draw_handshake_center",
                        joints={
                            "wrist_flex": clamp_joint_angle(
                                controller_config,
                                "wrist_flex",
                                center,
                            )
                        },
                        duration_s=center_move_s,
                    )
                )
            steps.extend(
                SettlementStep(
                    label=f"draw_handshake_{index + 1}",
                    joints={
                        "wrist_flex": clamp_joint_angle(
                            controller_config,
                            "wrist_flex",
                            center + (delta if index % 2 == 0 else -delta),
                        )
                    },
                    duration_s=duration_s / count,
                )
                for index in range(count)
            )
            return steps

        if action_name == "defeat_nod":
            target_flex = clamp_joint_angle(
                controller_config,
                "wrist_flex",
                float(current_angles["wrist_flex"]) + float(action.get("wrist_flex_delta_deg", 135.0)),
            )
            move_s = max(0.0, float(action.get("wrist_flex_move_s", 0.0)))
            steps = []
            if move_s > 0.0:
                steps.append(
                    SettlementStep(
                        label="defeat_wrist_flex_move",
                        joints={"wrist_flex": target_flex},
                        duration_s=move_s,
                    )
                )
            steps.append(
                SettlementStep(
                    label="defeat_wrist_flex_hold",
                    joints={"wrist_flex": target_flex},
                    duration_s=duration_s,
                )
            )
            return steps

        raise ValueError(f"未知结算动作: {action_name}")


def clamp_joint_angle(controller_config: dict, joint_name: str, angle: float) -> float:
    joint = controller_config["_joint_by_name"][joint_name]
    return min(max(float(angle), float(joint["angle_min_deg"])), float(joint["angle_max_deg"]))


def configured_home_angles(controller, current_angles: dict[str, float]) -> dict[str, float]:
    configured = controller.home_pose.get("joint_angles_deg", {})
    angles = dict(current_angles)
    for joint_name, angle in configured.items():
        angles[joint_name] = float(angle)
    return angles


def interpolate_joint_angles(start: dict[str, float], end: dict[str, float], steps: int):
    for index in range(1, int(steps) + 1):
        t = index / float(steps)
        s = t * t * (3.0 - 2.0 * t)
        yield {
            joint: float(start[joint]) + (float(end[joint]) - float(start[joint])) * s
            for joint in end
        }


def angle_to_raw_safe(controller_config: dict, joint_name: str, angle: float) -> int:
    from send_absolute_pose_template import angle_to_raw  # type: ignore

    joint = controller_config["_joint_by_name"][joint_name]
    return angle_to_raw(joint, clamp_joint_angle(controller_config, joint_name, float(angle)))


def angle_to_raw_direct(controller_config: dict, joint_name: str, angle: float) -> int:
    from send_absolute_pose_template import angle_to_raw  # type: ignore

    joint = controller_config["_joint_by_name"][joint_name]
    return angle_to_raw(joint, float(angle))


def make_direct_point(label: str, angles: dict[str, float], raw: dict[str, int], speed: int, acc: int, dt: float):
    from core.types import Pose6D, TrajectoryPoint  # type: ignore

    return TrajectoryPoint(
        phase=label,
        pose=Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "joint_direct"),
        angles=dict(angles),
        raw=dict(raw),
        speed=int(speed),
        acc=int(acc),
        dt=float(dt),
    )
