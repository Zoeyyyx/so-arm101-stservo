"""设置 ID5 / wrist_roll 的中位和软件限位。

这个工具只处理 wrist_roll，用于解决当前实物 ID5 有机械限位的问题。
写入前会先预览，真正写入需要加 `--yes`。
"""

import argparse
import json
import os
from datetime import datetime
from pathlib import Path

# Windows + conda-forge 数值库可能触发 OpenMP/DLL 加载顺序问题。
# 先设置环境变量并导入 torch，再导入 LeRobot。
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import torch  # noqa: F401

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SOFollower


WRIST_ROLL = "wrist_roll"
MODEL_MIDPOINT = 2047
MODEL_MAX_POSITION = 4095
RAW_PER_DEGREE = MODEL_MAX_POSITION / 360


def calibration_path(robot_id):
    # LeRobot 默认把 SO follower 的标定文件放在用户目录的 HuggingFace cache 里。
    return (
        Path.home()
        / ".cache"
        / "huggingface"
        / "lerobot"
        / "calibration"
        / "robots"
        / "so_follower"
        / f"{robot_id}.json"
    )


def degrees_to_centered_raw(degrees):
    # LeRobot 的 STS3215 degree 模式约等于 4095 raw 对应 360 度。
    return round(MODEL_MIDPOINT + degrees * RAW_PER_DEGREE)


def clamp_raw(value):
    return max(0, min(MODEL_MAX_POSITION, int(value)))


def main():
    parser = argparse.ArgumentParser(
        description="把当前 ID5 / wrist_roll 的物理姿态设为 LeRobot 0 度，并写入更窄的软件上下限。"
    )
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--id", default="soarm101_follower")
    parser.add_argument(
        "--min-deg",
        type=float,
        default=-45.0,
        help="wrist_roll 允许的最小角度。默认 -45，比完整 -180 安全很多。",
    )
    parser.add_argument(
        "--max-deg",
        type=float,
        default=15.0,
        help="wrist_roll 允许的最大角度。默认 +15，因为你当前 max 方向更容易碰限位。",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="真正写入舵机和 calibration 文件；不加时只预览将要写入的值。",
    )
    args = parser.parse_args()

    if args.min_deg >= args.max_deg:
        raise ValueError("--min-deg 必须小于 --max-deg")

    range_min = clamp_raw(degrees_to_centered_raw(args.min_deg))
    range_max = clamp_raw(degrees_to_centered_raw(args.max_deg))
    if range_min >= range_max:
        raise ValueError("换算后的 range_min 必须小于 range_max")

    calib_file = calibration_path(args.id)
    if not calib_file.exists():
        raise FileNotFoundError(f"找不到 calibration 文件: {calib_file}")

    calibration = json.loads(calib_file.read_text(encoding="utf-8"))
    if WRIST_ROLL not in calibration:
        raise KeyError(f"calibration 文件里没有 {WRIST_ROLL}")

    config = SO101FollowerConfig(port=args.port, id=args.id)
    robot = SOFollower(config)

    try:
        robot.bus.connect()

        # 直接从舵机读取当前写入的标定值，避免只相信文件里的旧值。
        motor_calibration = robot.bus.read_calibration()
        old_offset = int(motor_calibration[WRIST_ROLL].homing_offset)
        current_position = int(robot.bus.read("Present_Position", WRIST_ROLL, normalize=False))

        # Feetech/STS 的关系可理解为：
        #   Present_Position = Actual_Position - Homing_Offset
        # 所以 Actual_Position = 当前读数 + 旧 offset。
        # 要让当前位置变成 2047，也就是 LeRobot 的 0 度中位：
        #   新 offset = Actual_Position - 2047。
        actual_position = current_position + old_offset
        new_offset = actual_position - MODEL_MIDPOINT

        print(f"calibration file: {calib_file}")
        print(f"{WRIST_ROLL} old homing_offset: {old_offset}")
        print(f"{WRIST_ROLL} current Present_Position: {current_position}")
        print(f"{WRIST_ROLL} estimated Actual_Position: {actual_position}")
        print(f"{WRIST_ROLL} new homing_offset: {new_offset}")
        print(f"{WRIST_ROLL} requested degree limits: {args.min_deg:.1f} .. {args.max_deg:.1f}")
        print(f"{WRIST_ROLL} raw position limits: {range_min} .. {range_max}")

        if not args.yes:
            print("Dry run only. Add --yes to write this center and limit setting.")
            return

        # 写 EEPROM 类参数前先关闭扭矩。这里只处理 wrist_roll，不影响其他关节。
        robot.bus.disable_torque(WRIST_ROLL)
        robot.bus.write("Homing_Offset", WRIST_ROLL, new_offset, normalize=False)
        robot.bus.write("Min_Position_Limit", WRIST_ROLL, range_min, normalize=False)
        robot.bus.write("Max_Position_Limit", WRIST_ROLL, range_max, normalize=False)

        backup_file = calib_file.with_suffix(
            ".before_wrist_roll_center_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".json"
        )
        backup_file.write_text(json.dumps(calibration, indent=4), encoding="utf-8")

        calibration[WRIST_ROLL]["homing_offset"] = new_offset
        calibration[WRIST_ROLL]["range_min"] = range_min
        calibration[WRIST_ROLL]["range_max"] = range_max
        calib_file.write_text(json.dumps(calibration, indent=4), encoding="utf-8")

        checked_position = int(robot.bus.read("Present_Position", WRIST_ROLL, normalize=False))
        print(f"backup file: {backup_file}")
        print(f"{WRIST_ROLL} checked Present_Position after write: {checked_position}")
        print("Done. 当前 wrist_roll 物理姿态已经设置为 LeRobot 0 度，并写入了更窄的上下限。")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect()


if __name__ == "__main__":
    main()
