"""低层单舵机小幅点动工具。

只用于排查某一个 ID 的硬件运动是否正常；常规控制优先使用 LeRobot。
"""

import argparse
import time

from stservo_common import (
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    POSITION_MAX,
    POSITION_MIN,
    SERVO_IDS,
    STS_MODE,
    clamp,
    open_bus,
)


def read_position(packet_handler, scs_id):
    # 读取当前原始位置。这里是只读操作，不会让舵机运动。
    position, speed, result, error = packet_handler.ReadPosSpeed(scs_id)
    if result != COMM_SUCCESS:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getTxRxResult(result)))
    if error != 0:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getRxPacketError(error)))
    return position


def read_mode(packet_handler, scs_id):
    # 读取控制模式。普通位置模式通常是 0；轮模式/连续旋转模式通常不是 0。
    mode, result, error = packet_handler.read1ByteTxRx(scs_id, STS_MODE)
    if result != COMM_SUCCESS:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getTxRxResult(result)))
    if error != 0:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getRxPacketError(error)))
    return mode


def write_position(packet_handler, scs_id, position, speed, acc):
    # 发送一次目标位置命令。调用这个函数才会让舵机运动。
    result, error = packet_handler.WritePosEx(scs_id, int(position), speed, acc)
    if result != COMM_SUCCESS:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getTxRxResult(result)))
    if error != 0:
        raise RuntimeError("[ID:%03d] %s" % (scs_id, packet_handler.getRxPacketError(error)))


parser = argparse.ArgumentParser(description="低层单舵机点动工具，主要用于硬件排障。")
parser.add_argument("--port", default=DEFAULT_PORT)  # 微雪驱动板使用的串口号。
parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)  # STS3215 默认波特率。
parser.add_argument("--id", type=int, required=True, choices=SERVO_IDS)  # 要测试的舵机 ID。
parser.add_argument("--delta", type=int, default=40)  # 相对当前位置移动多少原始刻度。
parser.add_argument("--speed", type=int, default=120)  # 舵机速度参数。
parser.add_argument("--acc", type=int, default=10)  # 舵机加速度参数。
parser.add_argument("--hold", type=float, default=0.7)  # 到达目标后停留多久再回位。
parser.add_argument("--yes", action="store_true")  # 安全开关：不加 --yes 时只预演，不会运动。
args = parser.parse_args()

portHandler, packetHandler = open_bus(args.port, args.baudrate)

try:
    mode = read_mode(packetHandler, args.id)
    print("[ID:%03d] mode=%d" % (args.id, mode))
    if mode != 0:
        print("拒绝点动：该舵机不在位置模式。请先检查控制模式。")
        raise SystemExit(0)

    home = read_position(packetHandler, args.id)
    target = clamp(home + args.delta, POSITION_MIN, POSITION_MAX)

    print("[ID:%03d] home=%d target=%d return=%d speed=%d acc=%d" % (
        args.id,
        home,
        target,
        home,
        args.speed,
        args.acc,
    ))

    if not args.yes:
        print("当前只是预览。确认安全后追加 --yes 才会运动。")
        raise SystemExit(0)

    write_position(packetHandler, args.id, target, args.speed, args.acc)
    time.sleep(args.hold)
    after_target = read_position(packetHandler, args.id)
    print("[ID:%03d] after target position=%d" % (args.id, after_target))

    write_position(packetHandler, args.id, home, args.speed, args.acc)
    time.sleep(args.hold)
    after_return = read_position(packetHandler, args.id)
    print("[ID:%03d] after return position=%d" % (args.id, after_return))
    print("[ID:%03d] 点动完成" % args.id)
except SystemExit:
    pass
except RuntimeError as exc:
    print(exc)
finally:
    portHandler.closePort()
