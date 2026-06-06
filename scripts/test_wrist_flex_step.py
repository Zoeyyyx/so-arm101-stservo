"""Diagnose one STS3215 joint raw step motion on the bus.

By default this diagnoses wrist_flex, but --joint or --id can be used for the
same single-servo test on another joint. It reuses the existing Waveshare
STServo helper code and does not go through the arm motion planner, so it can
help separate hardware/limit issues from trajectory planning issues.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
STSERVO_TOOLS = REPO_ROOT / "tools" / "stservo"
if str(STSERVO_TOOLS) not in sys.path:
    sys.path.insert(0, str(STSERVO_TOOLS))


DEFAULT_TARGETS = [2500, 2800, 3000]
DEFAULT_JOINT = "wrist_flex"
DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 1000000
POSITION_MIN = 0
POSITION_MAX = 4095

STS_MODE = None
check_comm = None
open_bus = None


def load_stservo_common() -> None:
    global STS_MODE, check_comm, open_bus

    from stservo_common import (  # noqa: E402
        POSITION_MAX as sdk_position_max,
        POSITION_MIN as sdk_position_min,
        STS_MODE as sdk_sts_mode,
        check_comm as sdk_check_comm,
        open_bus as sdk_open_bus,
    )

    if sdk_position_min != POSITION_MIN or sdk_position_max != POSITION_MAX:
        raise RuntimeError(
            f"Unexpected SDK position range: {sdk_position_min}..{sdk_position_max}"
        )
    STS_MODE = sdk_sts_mode
    check_comm = sdk_check_comm
    open_bus = sdk_open_bus


def clamp_raw(value: int) -> int:
    return max(POSITION_MIN, min(POSITION_MAX, int(value)))


def load_servo_id(joint_name: str, servo_map_path: Path) -> int:
    data = json.loads(servo_map_path.read_text(encoding="utf-8"))
    for item in data.get("servos", []):
        if item.get("joint") == joint_name:
            return int(item["id"])
    raise RuntimeError(f"Cannot find joint {joint_name!r} in {servo_map_path}")


def load_joint_name(scs_id: int, servo_map_path: Path) -> str:
    data = json.loads(servo_map_path.read_text(encoding="utf-8"))
    for item in data.get("servos", []):
        if int(item.get("id")) == int(scs_id):
            return str(item.get("joint", f"id_{scs_id}"))
    return f"id_{scs_id}"


def read_mode(packet_handler, scs_id: int) -> int:
    mode, result, error = packet_handler.read1ByteTxRx(scs_id, STS_MODE)
    check_comm(packet_handler, scs_id, result, error, "read STS_MODE")
    return int(mode)


def read_position(packet_handler, scs_id: int) -> int:
    position, _speed, result, error = packet_handler.ReadPosSpeed(scs_id)
    check_comm(packet_handler, scs_id, result, error, "ReadPosSpeed")
    return int(position)


def write_position(packet_handler, scs_id: int, target_raw: int, speed: int, acc: int) -> None:
    result, error = packet_handler.WritePosEx(scs_id, int(target_raw), int(speed), int(acc))
    check_comm(packet_handler, scs_id, result, error, "WritePosEx")


def run_step(
    packet_handler,
    scs_id: int,
    target_raw: int,
    speed: int,
    acc: int,
    poll_interval_s: float,
    timeout_s: float,
    tolerance_raw: int,
    verbose: bool,
) -> dict:
    start_raw = read_position(packet_handler, scs_id)
    target_raw = clamp_raw(target_raw)

    print(
        f"[ID:{scs_id:03d}] start_raw={start_raw} target_raw={target_raw} "
        f"speed={speed} acc={acc}"
    )
    write_position(packet_handler, scs_id, target_raw, speed, acc)

    start_time = time.monotonic()
    deadline = start_time + float(timeout_s)
    final_raw = start_raw
    timed_out = False
    samples = []

    while True:
        now = time.monotonic()
        final_raw = read_position(packet_handler, scs_id)
        elapsed = now - start_time
        error = abs(final_raw - target_raw)
        samples.append((elapsed, final_raw, error))

        if verbose:
            print(f"  t={elapsed:6.3f}s raw={final_raw:4d} error={error:4d}")

        if error <= int(tolerance_raw):
            break
        if now >= deadline:
            timed_out = True
            break
        time.sleep(float(poll_interval_s))

    elapsed_time = time.monotonic() - start_time
    final_error = abs(final_raw - target_raw)
    return {
        "start_raw": start_raw,
        "target_raw": target_raw,
        "final_raw": final_raw,
        "final_error": final_error,
        "elapsed_time": elapsed_time,
        "timed_out": timed_out,
        "sample_count": len(samples),
    }


def print_result(result: dict, tolerance_raw: int) -> None:
    status = "OK" if (not result["timed_out"] and result["final_error"] <= tolerance_raw) else "NOT_REACHED"
    print(
        f"  result={status} start_raw={result['start_raw']} "
        f"target_raw={result['target_raw']} final_raw={result['final_raw']} "
        f"final_error={result['final_error']} elapsed_time={result['elapsed_time']:.3f}s "
        f"timed_out={result['timed_out']}"
    )


def print_conclusion(joint_name: str, results: list[dict], tolerance_raw: int, large_error_raw: int) -> None:
    reached_all = all((not item["timed_out"] and item["final_error"] <= tolerance_raw) for item in results)
    near_3000 = [item for item in results if abs(int(item["target_raw"]) - 3000) <= 80]
    bad_near_3000 = any(
        item["timed_out"] or int(item["final_error"]) >= int(large_error_raw)
        for item in near_3000
    )

    print("\n诊断结论:")
    if reached_all:
        print(
            f"2500/2800/3000 都能到位：{joint_name} 本身大概率正常，"
            "问题更可能是轨迹速度、插值节奏或 wait 策略。"
        )
    elif bad_near_3000:
        print(
            "3000 附近到不了或误差长期很大：可能存在机械阻力、供电不足、"
            "装配问题、标定问题或接近限位。"
        )
    else:
        print(
            "部分目标未稳定到位，但问题不只集中在 3000 附近；建议结合每段 "
            "final_error/timeout 继续检查通信、负载和目标范围。"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Single-servo raw-position step diagnostic."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Waveshare bus adapter serial port.")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-map", default=str(REPO_ROOT / "config" / "servo_map.json"))
    parser.add_argument("--joint", default=DEFAULT_JOINT, help="Joint name from servo_map.json.")
    parser.add_argument("--id", type=int, help="Servo ID override. Example: --id 2 tests joint 2.")
    parser.add_argument("--targets", type=int, nargs="+", default=DEFAULT_TARGETS)
    parser.add_argument("--speed", type=int, default=60, help="Default slow mode speed.")
    parser.add_argument("--acc", type=int, default=5, help="Default slow mode acceleration.")
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--tolerance-raw", type=int, default=30)
    parser.add_argument("--large-error-raw", type=int, default=120)
    parser.add_argument("--verbose", action="store_true", help="Print every 0.05s raw sample.")
    parser.add_argument("--execute", action="store_true", help="Actually move the selected joint.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    servo_map_path = Path(args.servo_map)
    if not servo_map_path.is_absolute():
        servo_map_path = REPO_ROOT / servo_map_path

    if args.id is None:
        joint_name = args.joint
        scs_id = load_servo_id(joint_name, servo_map_path)
    else:
        scs_id = int(args.id)
        joint_name = load_joint_name(scs_id, servo_map_path)
    targets = [clamp_raw(target) for target in args.targets]

    print("single-joint step diagnostic")
    print(f"  joint={joint_name} id={scs_id} servo_map={servo_map_path}")
    print(f"  port={args.port} baudrate={args.baudrate}")
    print(f"  targets={targets} speed={args.speed} acc={args.acc}")
    print(f"  poll_interval={args.poll_interval:.3f}s timeout={args.timeout:.3f}s")

    load_stservo_common()
    port_handler, packet_handler = open_bus(args.port, args.baudrate)
    try:
        mode = read_mode(packet_handler, scs_id)
        current_raw = read_position(packet_handler, scs_id)
        print(f"  current_raw={current_raw} mode={mode}")

        if mode != 0:
            print("拒绝执行：wrist_flex 不在位置模式。请先检查舵机控制模式。")
            return 1

        if not args.execute:
            print("\n当前是 dry-run：只读取当前位置，不发送运动指令。加 --execute 才会真实运行。")
            for target in targets:
                print(
                    f"  plan start_raw={current_raw} target_raw={target} "
                    f"speed={args.speed} acc={args.acc}"
                )
            return 0

        print(f"\n开始执行：只控制 {joint_name}，其他关节不发送指令。")
        results = []
        for target in targets:
            result = run_step(
                packet_handler=packet_handler,
                scs_id=scs_id,
                target_raw=target,
                speed=args.speed,
                acc=args.acc,
                poll_interval_s=args.poll_interval,
                timeout_s=args.timeout,
                tolerance_raw=args.tolerance_raw,
                verbose=args.verbose,
            )
            results.append(result)
            print_result(result, args.tolerance_raw)
            time.sleep(0.25)

        print_conclusion(joint_name, results, args.tolerance_raw, args.large_error_raw)
        return 0
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
