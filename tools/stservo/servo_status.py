"""读取单个 STS3215 舵机状态。

包括模式、扭矩、电压、温度、当前位置等信息，不发送运动指令。
"""

import argparse

from stservo_common import (
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    SERVO_IDS,
    STS_MODE,
    STS_PRESENT_TEMPERATURE,
    STS_PRESENT_VOLTAGE,
    STS_TORQUE_ENABLE,
    open_bus,
)


parser = argparse.ArgumentParser(description="读取单个 STS3215 舵机状态，不会发送运动指令。")
parser.add_argument("--port", default=DEFAULT_PORT)  # 微雪驱动板使用的串口号。
parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)  # STS3215 默认波特率。
parser.add_argument("--id", type=int, required=True, choices=SERVO_IDS)
args = parser.parse_args()

portHandler, packetHandler = open_bus(args.port, args.baudrate)

try:
    mode, result, error = packetHandler.read1ByteTxRx(args.id, STS_MODE)
    if result != COMM_SUCCESS:
        print("[ID:%03d] mode read failed: %s" % (args.id, packetHandler.getTxRxResult(result)))
        quit()

    torque, result, error = packetHandler.read1ByteTxRx(args.id, STS_TORQUE_ENABLE)
    voltage, _, _ = packetHandler.read1ByteTxRx(args.id, STS_PRESENT_VOLTAGE)
    temperature, _, _ = packetHandler.read1ByteTxRx(args.id, STS_PRESENT_TEMPERATURE)
    moving, _, _ = packetHandler.ReadMoving(args.id)
    position, speed, result, error = packetHandler.ReadPosSpeed(args.id)

    print("[ID:%03d] mode=%d" % (args.id, mode))
    print("[ID:%03d] torque_enable=%d" % (args.id, torque))
    print("[ID:%03d] position=%d speed=%d moving=%d" % (args.id, position, speed, moving))
    print("[ID:%03d] voltage_raw=%d temperature_c=%d" % (args.id, voltage, temperature))
finally:
    portHandler.closePort()
