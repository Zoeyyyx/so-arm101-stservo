"""关闭 STS3215 舵机扭矩。

出现卡滞、碰撞风险或需要手动调整姿态时，可以用它释放舵机保持力。
"""

import argparse

from stservo_common import (
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    SERVO_IDS,
    STS_TORQUE_ENABLE,
    open_bus,
)


parser = argparse.ArgumentParser(description="关闭一个或多个 STS3215 舵机扭矩。")
parser.add_argument("--port", default=DEFAULT_PORT)  # 微雪驱动板使用的串口号。
parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)  # STS3215 默认波特率。
parser.add_argument("--ids", type=int, nargs="+", default=SERVO_IDS)
args = parser.parse_args()

portHandler, packetHandler = open_bus(args.port, args.baudrate)

try:
    for scs_id in args.ids:
        # 写 0 到扭矩使能寄存器，释放该舵机的保持力。
        result, error = packetHandler.write1ByteTxRx(scs_id, STS_TORQUE_ENABLE, 0)
        if result != COMM_SUCCESS:
            print("[ID:%03d] %s" % (scs_id, packetHandler.getTxRxResult(result)))
            continue
        if error != 0:
            print("[ID:%03d] %s" % (scs_id, packetHandler.getRxPacketError(error)))
            continue
        print("[ID:%03d] 已关闭扭矩" % scs_id)
finally:
    portHandler.closePort()
