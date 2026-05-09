import argparse
import math
from pathlib import Path
import time
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "vendor" / "waveshare_stservo"))
from scservo_sdk import *

BAUDRATE = 1000000
POSITION_MIN = 0
POSITION_MAX = 4095


def clamp(value, low, high):
    return max(low, min(high, value))


def read_position(packet_handler, scs_id):
    position, speed, result, error = packet_handler.ReadPosSpeed(scs_id)
    if result != COMM_SUCCESS:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getTxRxResult(result)))
    if error != 0:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getRxPacketError(error)))
    return position


def write_position(packet_handler, scs_id, position, speed, acc):
    result, error = packet_handler.WritePosEx(scs_id, int(position), speed, acc)
    if result != COMM_SUCCESS:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getTxRxResult(result)))
    if error != 0:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getRxPacketError(error)))


parser = argparse.ArgumentParser(description="Smooth small wave demo for the first installed SO-ARM servos.")
parser.add_argument("--port", default="COM5")
parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3])
parser.add_argument("--amp", type=int, default=220)
parser.add_argument("--amp1", type=int)
parser.add_argument("--amp2", type=int)
parser.add_argument("--amp3", type=int)
parser.add_argument("--amp4", type=int)
parser.add_argument("--center1", type=int, default=0)
parser.add_argument("--center2", type=int, default=0)
parser.add_argument("--center3", type=int, default=0)
parser.add_argument("--center4", type=int, default=0)
parser.add_argument("--cycles", type=float, default=1.0)
parser.add_argument("--steps", type=int, default=90)
parser.add_argument("--dt", type=float, default=0.04)
parser.add_argument("--speed", type=int, default=180)
parser.add_argument("--acc", type=int, default=20)
parser.add_argument("--yes", action="store_true")
args = parser.parse_args()

for scs_id in args.ids:
    if scs_id not in [1, 2, 3, 4]:
        raise SystemExit("Only IDs 1-4 are allowed.")

portHandler = PortHandler(args.port)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    print("Failed to open the port")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set the baudrate")
    portHandler.closePort()
    quit()

home = {}
phases = {1: 0.0, 2: 1.4, 3: 2.8, 4: 4.2}
amp_overrides = {
    1: args.amp1,
    2: args.amp2,
    3: args.amp3,
    4: args.amp4,
}
center_offsets = {
    1: args.center1,
    2: args.center2,
    3: args.center3,
    4: args.center4,
}

try:
    for scs_id in args.ids:
        home[scs_id] = read_position(packetHandler, scs_id)

    print("Home positions and safe amplitudes:")
    safe_amp = {}
    for scs_id in args.ids:
        requested_amp = args.amp if amp_overrides[scs_id] is None else amp_overrides[scs_id]
        home[scs_id] = clamp(home[scs_id] + center_offsets[scs_id], POSITION_MIN, POSITION_MAX)
        margin = min(home[scs_id] - POSITION_MIN, POSITION_MAX - home[scs_id])
        safe_amp[scs_id] = max(0, min(requested_amp, margin - 20))
        print("[ID:%03d] center=%d amp=%d" % (scs_id, home[scs_id], safe_amp[scs_id]))

    if not args.yes:
        print("Dry run only. Add --yes to move the servos.")
        raise SystemExit(0)

    for step in range(args.steps + 1):
        theta = 2.0 * math.pi * args.cycles * step / args.steps
        for scs_id in args.ids:
            offset = safe_amp[scs_id] * math.sin(theta + phases[scs_id])
            target = clamp(home[scs_id] + offset, POSITION_MIN, POSITION_MAX)
            write_position(packetHandler, scs_id, target, args.speed, args.acc)
        time.sleep(args.dt)

    for scs_id in args.ids:
        write_position(packetHandler, scs_id, home[scs_id], args.speed, args.acc)
    time.sleep(0.5)
    print("smooth wave complete")
except SystemExit:
    pass
except RuntimeError as exc:
    print(exc)
finally:
    portHandler.closePort()
