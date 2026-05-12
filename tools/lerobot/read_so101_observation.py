"""读取 SO101 follower 当前关节状态。

这个脚本只读 observation，不发送动作，适合作为每次上电后的第一步检查。
"""

import argparse
import os
import time

# Windows + conda-forge 数值库可能触发 OpenMP/DLL 加载顺序问题。
# 先设置环境变量并导入 torch，再导入 LeRobot。
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import torch  # noqa: F401

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SOFollower


parser = argparse.ArgumentParser(description="只读取 SO-101 follower 状态，不发送动作。")
parser.add_argument("--port", default="COM5")  # 微雪驱动板使用的串口号。
parser.add_argument("--id", default="soarm101_follower")  # LeRobot calibration 使用的 robot.id。
parser.add_argument("--count", type=int, default=10)  # 读取次数。
parser.add_argument("--dt", type=float, default=0.5)  # 每次读取之间的延时，单位是秒。
args = parser.parse_args()

# 这里使用 LeRobot 自己的 SO101 follower 配置和 calibration 文件。
config = SO101FollowerConfig(port=args.port, id=args.id)
robot = SOFollower(config)

try:
    robot.connect()
    for index in range(args.count):
        obs = robot.get_observation()
        print("读取轮次 %d/%d" % (index + 1, args.count))
        for key, value in obs.items():
            if key.endswith(".pos"):
                print("%s = %.3f" % (key, value))
        print("-" * 32)
        time.sleep(args.dt)
finally:
    if robot.is_connected:
        robot.disconnect()
