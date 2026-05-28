"""SO101 打靶控制核心模块。

这里放可被 CLI 和后续 ROS 节点复用的逻辑。核心模块不直接打印、不调用
SystemExit，方便上层根据 PlanResult 决定如何响应。
"""

