"""打靶控制共享数据类型。"""

from dataclasses import dataclass, field
from typing import Any


IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
ALL_JOINTS = IK_JOINTS + ["gripper"]

# 打靶动作中暂时只用前四个关节完成伸出和下击。
# wrist_roll 有机械限位，gripper 只作为末端执行器保持当前状态，不参与规划和越界检查。
PASSIVE_JOINTS = ["wrist_roll", "gripper"]
ACTIVE_JOINTS = [joint for joint in ALL_JOINTS if joint not in PASSIVE_JOINTS]


@dataclass
class Pose6D:
    """末端绝对位姿，位置单位米，姿态单位度。"""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    frame: str


@dataclass
class JointState:
    """从总线读到的当前关节状态。"""

    angles: dict[str, float]
    raw: dict[str, int]


@dataclass
class MotionProfile:
    """某个动作阶段的舵机运动参数。"""

    steps: int
    speed: int
    acc: int
    dt: float


@dataclass
class TrajectoryPoint:
    """一帧已经通过 IK 和安全检查的关节目标。"""

    phase: str
    pose: Pose6D
    angles: dict[str, float]
    raw: dict[str, int]
    position_error_mm: float = 0.0
    achieved_position_m: list[float] | None = None
    orientation_error_deg: float | None = None
    orientation_requested: bool = False
    orientation_fallback: bool = False
    speed: int = 100
    acc: int = 10
    dt: float = 0.04


@dataclass
class PlanResult:
    """规划结果。success=False 时不应执行 trajectory。"""

    success: bool
    reason: str = ""
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    poses: dict[str, Pose6D] = field(default_factory=dict)
    diagnostics: dict[str, Any] = field(default_factory=dict)
