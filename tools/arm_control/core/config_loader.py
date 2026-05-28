"""配置加载与坐标系转换。"""

from __future__ import annotations

import json
from pathlib import Path

from core.types import Pose6D
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"
DEFAULT_READY_CONFIG = REPO_ROOT / "config" / "ready_pose.json"
DEFAULT_FORBIDDEN_ZONE = REPO_ROOT / "config" / "forbidden_zone.json"


def resolve_path(path) -> Path:
    """把相对路径解析到项目根目录下。"""
    result = Path(path)
    if not result.is_absolute():
        result = REPO_ROOT / result
    return result


def load_json(path) -> dict:
    """读取 JSON，并附带 _path 字段便于排障。"""
    json_path = resolve_path(path)
    data = json.loads(json_path.read_text(encoding="utf-8"))
    data["_path"] = str(json_path)
    return data


def load_hit_config(path=DEFAULT_HIT_CONFIG) -> dict:
    """读取打靶动作配置。"""
    return load_json(path)


def load_controller_config(hit_config: dict) -> dict:
    """读取底层舵机、URDF、IK 配置。"""
    from send_absolute_pose_template import load_config

    return load_config(resolve_path(hit_config["controller_config"]))


def load_home_pose(hit_config: dict, home_config_path=None) -> dict:
    """读取 home/stow 姿态。

    优先使用 config/home_pose.json；若旧项目还没有该文件，则兼容读取
    hit_action.json 内部的 home_pose。
    """
    if home_config_path is not None:
        return load_json(home_config_path)
    configured_path = hit_config.get("home_pose_config")
    if configured_path:
        return load_json(configured_path)
    if DEFAULT_HOME_CONFIG.exists():
        return load_json(DEFAULT_HOME_CONFIG)
    home_pose = dict(hit_config.get("home_pose", {}))
    home_pose["_path"] = f"{hit_config.get('_path', 'hit_action.json')}::home_pose"
    return home_pose


def load_ready_pose(hit_config: dict, ready_config_path=None, home_pose=None) -> dict:
    """读取 ready_pose 安全展开姿态。

    若没有 ready_pose 配置，则临时退回 home_pose，保证旧配置仍可运行；但实际
    打靶建议手动保存一个更适合展开的 ready_pose。
    """
    if ready_config_path is not None:
        return load_json(ready_config_path)
    configured_path = hit_config.get("ready_pose_config")
    if configured_path:
        return load_json(configured_path)
    if DEFAULT_READY_CONFIG.exists():
        return load_json(DEFAULT_READY_CONFIG)
    ready_pose = dict(home_pose or hit_config.get("home_pose", {}))
    ready_pose["_path"] = f"{hit_config.get('_path', 'hit_action.json')}::ready_pose_fallback_home"
    return ready_pose


def load_forbidden_zone(path=DEFAULT_FORBIDDEN_ZONE):
    """读取禁入区；文件不存在则返回 None。"""
    if path is None:
        return None
    zone_path = resolve_path(path)
    if not zone_path.exists():
        return None
    zone = json.loads(zone_path.read_text(encoding="utf-8"))
    zone["_path"] = str(zone_path)
    return zone


def transform_target_to_base(target_pose: Pose6D, hit_config: dict) -> Pose6D:
    """把输入坐标转换到机械臂 base 坐标系。

    当前默认输入已经是 so101_base。后续接 ROS/视觉时，可以把小车坐标或
    相机坐标通过 TF 转成 base 坐标后再调用控制器。
    """
    frames = hit_config["frames"]
    mount = hit_config["mobile_base_mount"]
    if target_pose.frame == frames["robot_base_frame"]:
        return Pose6D(
            target_pose.x,
            target_pose.y,
            target_pose.z,
            target_pose.roll,
            target_pose.pitch,
            target_pose.yaw,
            frames["robot_base_frame"],
        )
    if target_pose.frame == frames["cart_ground_frame"]:
        return Pose6D(
            target_pose.x - float(mount["base_offset_x_m"]),
            target_pose.y - float(mount["base_offset_y_m"]),
            target_pose.z - float(mount["base_height_above_ground_m"]),
            target_pose.roll,
            target_pose.pitch,
            target_pose.yaw,
            frames["robot_base_frame"],
        )
    raise ValueError(f"未知目标坐标系: {target_pose.frame}")


def scale_phase_profiles(hit_config: dict, speed_scale: float, acc_scale: float) -> None:
    """按比例缩放各阶段速度和加速度。"""
    for profile in hit_config["hit_action"]["phases"].values():
        profile["speed"] = max(1, int(round(float(profile["speed"]) * float(speed_scale))))
        profile["acc"] = max(1, int(round(float(profile["acc"]) * float(acc_scale))))
