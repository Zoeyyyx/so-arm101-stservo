"""读取 STS3215 原始位置和速度。

用于确认总线读取稳定，以及观察机械臂手动移动时的原始位置变化。
"""

import argparse
import time

from stservo_common import COMM_SUCCESS, DEFAULT_BAUDRATE, DEFAULT_PORT, SERVO_IDS, open_bus


parser = argparse.ArgumentParser(description="读取 STS3215 原始位置，不会发送运动指令。")
parser.add_argument("--port", default=DEFAULT_PORT)  # 微雪驱动板使用的串口号。
parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)  # STS3215 默认波特率。
parser.add_argument("--ids", type=int, nargs="+", default=SERVO_IDS)
parser.add_argument("--count", type=int, default=10)  # 读取轮数。
parser.add_argument("--dt", type=float, default=0.5)  # 每轮读取之间的延时，单位是秒。
args = parser.parse_args()

portHandler, packetHandler = open_bus(args.port, args.baudrate)

try:
    for cycle in range(args.count):
        print("读取轮次 %d/%d" % (cycle + 1, args.count))
        for scs_id in args.ids:
            # ReadPosSpeed 是只读操作：只读取当前位置和速度，不会命令舵机运动。
            position, speed, result, error = packetHandler.ReadPosSpeed(scs_id)
            if result != COMM_SUCCESS:
                print("[ID:%03d] %s" % (scs_id, packetHandler.getTxRxResult(result)))
                continue
            if error != 0:
                print("[ID:%03d] %s" % (scs_id, packetHandler.getRxPacketError(error)))
                continue
            print("[ID:%03d] Pos:%d Speed:%d" % (scs_id, position, speed))
        print("-" * 32)
        time.sleep(args.dt)
finally:
    portHandler.closePort()
