"""STServo 低层串口工具的共用代码。

这个文件只负责三件事：
1. 找到仓库内保存的微雪 STservo SDK；
2. 提供默认串口、波特率和舵机 ID；
3. 统一打开 STS3215 总线，避免每个脚本重复写一遍。

注意：这里使用的是微雪 `STservo_sdk` 包，不修改官方底层通信协议。
"""

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_ROOT = REPO_ROOT / "vendor" / "waveshare_stservo"

# 让 Python 可以从 vendor/ 目录导入微雪 SDK。
if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))

from STservo_sdk import *  # noqa: F401,F403,E402


DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 1000000
DEFAULT_OPEN_RETRIES = 3
DEFAULT_OPEN_RETRY_DELAY_S = 0.25
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# STS3215 的原始位置范围。LeRobot 标定后会转换成角度/归一化值。
POSITION_MIN = 0
POSITION_MAX = 4095


def clamp(value, low, high):
    """把数值限制在指定范围内，防止越界目标被发送给舵机。"""
    return max(low, min(high, value))


def open_bus(
    port=DEFAULT_PORT,
    baudrate=DEFAULT_BAUDRATE,
    retries=DEFAULT_OPEN_RETRIES,
    retry_delay_s=DEFAULT_OPEN_RETRY_DELAY_S,
):
    """打开串口并返回 `port_handler, packet_handler`。

    `PortHandler` 来自微雪 SDK，负责串口；
    `sts` 来自微雪 SDK，负责 ST 系列舵机协议。
    """
    import time

    last_error = None
    attempts = max(1, int(retries))
    for attempt in range(1, attempts + 1):
        port_handler = PortHandler(port)  # noqa: F405
        packet_handler = sts(port_handler)  # noqa: F405
        port_handler.baudrate = int(baudrate)

        try:
            if not port_handler.openPort():
                raise RuntimeError(f"无法打开串口: {port}")

            return port_handler, packet_handler
        except Exception as exc:
            last_error = exc
            try:
                if getattr(port_handler, "is_open", False):
                    port_handler.closePort()
            except Exception:
                pass
            if attempt < attempts:
                time.sleep(float(retry_delay_s))

    raise RuntimeError(f"无法打开串口: {port} baudrate={baudrate}") from last_error


def check_comm(packet_handler, scs_id, result, error, action):
    """检查一次 SDK 调用结果，失败时抛出带 ID 和动作名的异常。"""
    if result != COMM_SUCCESS:  # noqa: F405
        raise RuntimeError(
            "[ID:%03d] %s failed: %s"
            % (scs_id, action, packet_handler.getTxRxResult(result))
        )
    if error != 0:
        raise RuntimeError(
            "[ID:%03d] %s error: %s"
            % (scs_id, action, packet_handler.getRxPacketError(error))
        )
