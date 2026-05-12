"""扫描 STS3215 总线上的舵机 ID。

这是低层通信排查的第一步：只 ping，不会让舵机运动。
"""

import argparse

from stservo_common import COMM_SUCCESS, DEFAULT_BAUDRATE, DEFAULT_PORT, SERVO_IDS, open_bus


parser = argparse.ArgumentParser(description="扫描 STS3215 舵机 ID，不会让舵机运动。")
parser.add_argument("--port", default=DEFAULT_PORT)  # Windows 示例：COM5。
parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)  # STS3215 默认波特率。
parser.add_argument("--start", type=int, default=1)  # 开始扫描的 ID。
parser.add_argument("--end", type=int, default=max(SERVO_IDS))  # 结束扫描的 ID。
args = parser.parse_args()

portHandler, packetHandler = open_bus(args.port, args.baudrate)

found = []
for scs_id in range(args.start, args.end + 1):
    # ping() 是只读操作：只检查该 ID 是否回包，并读取型号编号，不会让舵机运动。
    model, result, error = packetHandler.ping(scs_id)
    if result == COMM_SUCCESS:
        found.append(scs_id)
        print("[ID:%03d] ping 成功，舵机型号编号: %d" % (scs_id, model))
        if error != 0:
            print("[ID:%03d] %s" % (scs_id, packetHandler.getRxPacketError(error)))
    else:
        print("[ID:%03d] %s" % (scs_id, packetHandler.getTxRxResult(result)))

portHandler.closePort()

if found:
    print("发现在线 ID:", ", ".join(str(scs_id) for scs_id in found))
else:
    print("在 ID 范围 %d-%d 内没有发现舵机" % (args.start, args.end))
