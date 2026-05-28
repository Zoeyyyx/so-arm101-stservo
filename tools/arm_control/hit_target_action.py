"""SO101 地面靶位打靶 CLI。

核心逻辑已经拆到 tools/arm_control/core/，本文件只负责：
1. 解析命令行参数；
2. 加载控制器；
3. 打印 dry-run/执行报告；
4. 在 --yes 时发送舵机轨迹。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HIT_CONFIG = REPO_ROOT / "config" / "hit_action.json"
DEFAULT_HOME_CONFIG = REPO_ROOT / "config" / "home_pose.json"
DEFAULT_READY_CONFIG = REPO_ROOT / "config" / "ready_pose.json"
DEFAULT_FORBIDDEN_ZONE = REPO_ROOT / "config" / "forbidden_zone.json"
ALL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def build_parser():
    """创建 CLI 参数。"""
    parser = argparse.ArgumentParser(description="SO101 地面靶位打靶动作。")
    parser.add_argument("--hit-config", default=str(DEFAULT_HIT_CONFIG), help="打靶动作配置。")
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG), help="home/stow 姿态配置。")
    parser.add_argument("--ready-config", default=str(DEFAULT_READY_CONFIG), help="ready/safe 展开姿态配置。")
    parser.add_argument("--forbidden-zone", default=None, help="可选：末端禁入区 JSON。默认不启用。")
    parser.add_argument("--port", help="覆盖 COM 口，例如 COM5。")
    parser.add_argument("--baudrate", type=int, help="覆盖波特率。")
    parser.add_argument("--frame", help="目标坐标系，默认来自 hit_action.json。")
    parser.add_argument("--x", type=float, required=True, help="靶心 x，单位米。")
    parser.add_argument("--y", type=float, required=True, help="靶心 y，单位米。")
    parser.add_argument("--z", type=float, required=True, help="靶面/接触点 z，单位米。")
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--strike-height", type=float, help="下击前靶位上方高度，单位米。")
    parser.add_argument("--above-height", type=float, help="兼容旧参数：等同于 --strike-height。")
    parser.add_argument("--hover-height", type=float, help="兼容旧参数：等同于 --strike-height。")
    parser.add_argument("--contact-offset", type=float, help="接触点 z 偏移，单位米。")
    parser.add_argument("--pre-strike-dwell", type=float, help="到靶位上方后的停留时间，单位秒。")
    parser.add_argument("--hit-hold", type=float, help="击中后的停留时间，单位秒。")
    parser.add_argument("--dwell", type=float, help="兼容旧参数：等同于 --hit-hold。")
    parser.add_argument("--gripper", type=float, help="兼容旧参数：当前打靶程序会保持夹爪不动。")
    parser.add_argument("--wrist-roll", type=float, help="兼容旧参数：当前打靶程序会保持 wrist_roll 不动。")
    parser.add_argument("--no-tool-down", action="store_true", help="不强制工具轴向下，仅做位置 IK。")
    parser.add_argument("--workspace-samples", type=int, help="覆盖工作空间采样数量。")
    parser.add_argument("--max-joint-step", type=float, help="相邻轨迹点最大单关节角度变化。")
    parser.add_argument("--approach-error-max-mm", type=float, default=None, help="approach_above_target 阶段允许的最大 IK 位置误差，默认 15。")
    parser.add_argument("--strike-error-max-mm", type=float, default=None, help="strike_down 阶段允许的最大 IK 位置误差，默认 15。")
    parser.add_argument("--home-tolerance", type=float, help="启动姿态与 home 的允许角度差，单位度。")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="整体缩放各阶段速度。")
    parser.add_argument("--acc-scale", type=float, default=1.0, help="整体缩放各阶段加速度。")
    parser.add_argument("--strict-servo-wait", action="store_true", help="开启逐点到位等待；排查轨迹跟随时使用。")
    parser.add_argument("--no-strict-servo-wait", action="store_true", help="关闭逐点到位等待。")
    parser.add_argument("--wait-raw-tolerance", type=int, help="逐点到位 raw 容差，默认来自配置。")
    parser.add_argument("--wait-timeout", type=float, help="每个轨迹点等待到位超时时间，单位秒。")
    parser.add_argument(
        "--wait-timeout-policy",
        choices=["warn", "error"],
        help="到位等待超时时的处理方式：warn=警告后继续，error=中断。",
    )
    parser.add_argument("--sync-write", action="store_true", help="开启 SyncWrite 同步写入；默认使用普通逐舵机写入。")
    parser.add_argument("--no-sync-write", action="store_true", help="关闭 SyncWrite。")
    parser.add_argument("--show-workspace", action="store_true", help="只打印安全包络和工作空间，不连接舵机。")
    parser.add_argument("--yes", action="store_true", help="真正执行动作；不加时只 dry-run。")
    return parser


def apply_runtime_overrides(controller, args):
    """把 CLI 临时参数写入本次运行的配置。"""
    hit_config = controller.hit_config
    if args.strike_height is not None:
        hit_config["hit_action"]["strike_height_m"] = args.strike_height
    if args.above_height is not None:
        hit_config["hit_action"]["strike_height_m"] = args.above_height
    if args.hover_height is not None:
        hit_config["hit_action"]["strike_height_m"] = args.hover_height
    if args.contact_offset is not None:
        hit_config["hit_action"]["contact_offset_m"] = args.contact_offset
    if args.pre_strike_dwell is not None:
        hit_config["hit_action"]["pre_strike_dwell_s"] = args.pre_strike_dwell
    if args.hit_hold is not None:
        hit_config["hit_action"]["hit_hold_s"] = args.hit_hold
    if args.dwell is not None:
        hit_config["hit_action"]["hit_hold_s"] = args.dwell
    if args.max_joint_step is not None:
        hit_config["hit_action"]["max_joint_step_deg"] = args.max_joint_step
    if args.approach_error_max_mm is not None:
        hit_config["hit_action"]["approach_error_max_mm"] = args.approach_error_max_mm
    if args.strike_error_max_mm is not None:
        hit_config["hit_action"]["strike_error_max_mm"] = args.strike_error_max_mm
    if args.gripper is not None:
        print("提示：当前打靶程序已禁用 gripper 规划，--gripper 会被忽略。")
    if args.wrist_roll is not None:
        print("提示：当前打靶程序已禁用 wrist_roll 规划，--wrist-roll 会被忽略。")
    if args.no_tool_down:
        hit_config["tool_orientation"]["enforce_tool_down"] = False
    if args.strict_servo_wait:
        hit_config["hit_action"]["strict_servo_wait"] = True
        hit_config["hit_action"]["debug_visible_strike"] = True
    if args.no_strict_servo_wait:
        hit_config["hit_action"]["strict_servo_wait"] = False
        hit_config["hit_action"]["debug_visible_strike"] = False
    if args.wait_raw_tolerance is not None:
        hit_config["hit_action"]["wait_raw_tolerance"] = args.wait_raw_tolerance
    if args.wait_timeout is not None:
        hit_config["hit_action"]["wait_timeout_s"] = args.wait_timeout
    if args.wait_timeout_policy is not None:
        hit_config["hit_action"]["wait_timeout_policy"] = args.wait_timeout_policy
    if args.sync_write:
        hit_config["hit_action"]["sync_write"] = True
    if args.no_sync_write:
        hit_config["hit_action"]["sync_write"] = False


def print_state(state):
    """打印当前 raw 和角度。"""
    print("当前 raw / angle:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} raw={state.raw[joint]:4d} angle={state.angles[joint]:8.3f} deg")


def print_trajectory_summary(result, controller=None):
    """打印规划轨迹摘要。"""
    from core.trajectory_planner import max_adjacent_joint_delta, max_joint_delta_degrees, phase_position_error_limit_mm

    return_policy = result.diagnostics.get("return_policy")
    if return_policy:
        print("回程策略:")
        print(f"  {return_policy}")
        if result.diagnostics.get("strict_reverse_return"):
            print(
                "  已启用严格反向："
                f"return_strike_down 点数={result.diagnostics.get('return_strike_down_points')}，"
                f"return_approach_above_target 点数={result.diagnostics.get('return_approach_above_target_points')}，"
                f"return_move_to_ready 点数={result.diagnostics.get('return_move_to_ready_points')}，"
                f"return_home 点数={result.diagnostics.get('return_home_points')}。"
            )
    reverse_check = result.diagnostics.get("reverse_check")
    if reverse_check:
        print("严格反向数据校验:")
        for title, key in [
            ("下击阶段 vs 下击反向阶段", "strike_down_vs_return_strike_down"),
            ("home+前伸阶段 vs 前伸反向+home 反向阶段", "home_approach_vs_return_stack"),
        ]:
            item = reverse_check[key]
            print(
                f"  {title}: ok={item['ok']} "
                f"forward_points={item['forward_points']} reverse_points={item['reverse_points']} "
                f"max_angle_error={item['max_angle_error_deg']:.12f} deg "
                f"max_raw_error={item['max_raw_error']}"
            )
            if item.get("first_mismatch"):
                print(f"    first_mismatch={item['first_mismatch']}")
    strike_contract = result.diagnostics.get("strike_contract")
    if strike_contract:
        print("strike_down 坐标合同校验:")
        print(
            f"  ok={strike_contract['ok']} points={strike_contract['strike_points']} "
            f"max_xy_error={strike_contract['max_xy_error_m']:.12f} m "
            f"max_position_error={strike_contract['max_position_error_mm']:.3f} mm"
        )
        print(f"  expected_above={strike_contract['expected_above']}")
        print(f"  expected_contact={strike_contract['expected_contact']}")
        if strike_contract.get("first_mismatch"):
            print(f"  first_mismatch={strike_contract['first_mismatch']}")
    adaptive_steps = result.diagnostics.get("adaptive_cartesian_steps")
    if adaptive_steps:
        print("笛卡尔阶段自动加密记录:")
        for phase, attempts in adaptive_steps.items():
            if not attempts:
                continue
            last = attempts[-1]
            print(
                f"  {phase}: final_steps={last['steps']} "
                f"max_step_delta={last['max_step_delta_deg']:.3f} deg "
                f"attempts={attempts}"
            )

    print("打靶末端位姿:")
    for name, pose in result.poses.items():
        print(f"  {name:14s}: x={pose.x:.4f} y={pose.y:.4f} z={pose.z:.4f} frame={pose.frame}")

    print("轨迹概要:")
    phase_names = []
    for point in result.trajectory:
        if point.phase not in phase_names:
            phase_names.append(point.phase)
    for phase in phase_names:
        points = [point for point in result.trajectory if point.phase == phase]
        if not points:
            continue
        finite_z = [point.pose.z for point in points if math.isfinite(point.pose.z)]
        max_position_error = max(point.position_error_mm for point in points)
        orientation_errors = [point.orientation_error_deg for point in points if point.orientation_error_deg is not None]
        max_orientation_error = max(orientation_errors) if orientation_errors else None
        fallback_count = sum(1 for point in points if point.orientation_fallback)
        phase_delta = max_joint_delta_degrees(points[0].angles, points[-1].angles) if len(points) > 1 else 0.0
        error_limit = None
        if controller is not None:
            phase_for_limit = phase
            if phase == "return_approach_above_target":
                phase_for_limit = "approach_above_target"
            elif phase == "return_strike_down":
                phase_for_limit = "strike_down"
            error_limit = phase_position_error_limit_mm(
                controller.hit_config,
                controller.controller_config,
                phase_for_limit,
            )
        print(
            f"  {phase:22s} points={len(points):3d} "
            f"speed={points[-1].speed:3d} acc={points[-1].acc:2d} dt={points[-1].dt:.3f} "
            f"duration={sum(point.dt for point in points):.2f}s "
            f"max_step_delta={max_adjacent_joint_delta(points):.3f}deg "
            f"phase_delta={phase_delta:.3f}deg max_pos_err={max_position_error:.3f}mm"
            + (f" limit={error_limit:.3f}mm" if error_limit is not None else "")
        )
        if finite_z:
            print(f"  {'':22s} z={min(finite_z):.4f}..{max(finite_z):.4f} m")
        if max_orientation_error is not None:
            print(f"  {'':22s} max_tool_axis_err={max_orientation_error:.3f} deg")
        if fallback_count:
            print(f"  {'':22s} tool_down_fallback={fallback_count} 点，已优先保证靶心位置")

    final = result.trajectory[-1]
    print("最终目标 raw:")
    for joint in ALL_JOINTS:
        print(f"  {joint:14s} raw={final.raw[joint]}")


def main():
    args = build_parser().parse_args()
    from core.arm_controller import ArmController
    from core.types import Pose6D
    from send_absolute_pose_template import print_joint_envelope, print_workspace_bounds

    forbidden_zone = args.forbidden_zone
    if forbidden_zone == "default":
        forbidden_zone = str(DEFAULT_FORBIDDEN_ZONE)

    controller = ArmController.from_files(
        args.hit_config,
        home_config_path=args.home_config,
        ready_config_path=args.ready_config,
        forbidden_zone_path=forbidden_zone,
        port=args.port,
        baudrate=args.baudrate,
        workspace_samples=args.workspace_samples,
        speed_scale=args.speed_scale,
        acc_scale=args.acc_scale,
        home_tolerance=args.home_tolerance,
    )
    apply_runtime_overrides(controller, args)

    print_joint_envelope(controller.controller_config, controller.safe_limits)
    print_workspace_bounds(controller.workspace_bounds)
    if args.show_workspace:
        return

    target_frame = args.frame or controller.hit_config["frames"]["default_target_frame"]
    target_pose = Pose6D(args.x, args.y, args.z, args.roll, args.pitch, args.yaw, target_frame)

    controller.connect()
    try:
        state = controller.read_state()
        result, target_pose_base = controller.plan_hit(target_pose, state, gripper_override=args.gripper)

        print("配置文件:")
        print(f"  hit_config={controller.hit_config.get('_path')}")
        print(f"  home_pose={controller.home_pose.get('_path')}")
        print(f"  ready_pose={controller.ready_pose.get('_path')}")
        print(f"  serial={controller.controller_config['serial']['port']} baudrate={controller.controller_config['serial']['baudrate']}")
        print_state(state)
        print("输入靶心:")
        print(f"  frame={target_pose.frame} x={target_pose.x:.4f} y={target_pose.y:.4f} z={target_pose.z:.4f}")
        print("机械臂基座坐标下靶心:")
        print(
            f"  frame={target_pose_base.frame} "
            f"x={target_pose_base.x:.4f} y={target_pose_base.y:.4f} z={target_pose_base.z:.4f}"
        )

        if not result.success:
            print("拒绝执行打靶动作：")
            print(result.reason)
            if result.diagnostics.get("reverse_check"):
                print("严格反向数据校验失败详情:")
                print(result.diagnostics["reverse_check"])
            if result.diagnostics.get("strike_contract"):
                print("strike_down 坐标合同失败详情:")
                print(result.diagnostics["strike_contract"])
            if result.diagnostics.get("home_differences"):
                print("下一步：先运行 return_home.py 低速回到 home，然后再运行 hit_target_action.py。")
            else:
                print("下一步：先根据上面的合同/反向校验详情修正目标坐标或轨迹生成逻辑。")
            return

        print_trajectory_summary(result, controller)
        print(
            "动作停留时间: "
            f"above停留={float(controller.hit_config['hit_action'].get('pre_strike_dwell_s', 0.0)):.2f}s "
            f"击中停留={float(controller.hit_config['hit_action'].get('hit_hold_s', 0.0)):.2f}s "
            f"回升停留={float(controller.hit_config['hit_action'].get('after_rise_dwell_s', 0.0)):.2f}s"
        )
        print(
            "执行模式: "
            f"sync_write={controller.hit_config['hit_action'].get('sync_write', False)} "
            f"strict_servo_wait={controller.hit_config['hit_action'].get('strict_servo_wait', False)} "
            f"debug_visible_strike={controller.hit_config['hit_action'].get('debug_visible_strike', False)} "
            f"wait_raw_tolerance={controller.hit_config['hit_action'].get('wait_raw_tolerance', 25)} "
            f"wait_timeout_s={controller.hit_config['hit_action'].get('wait_timeout_s', 0.8)} "
            f"wait_timeout_policy={controller.hit_config['hit_action'].get('wait_timeout_policy', 'warn')}"
        )

        if not args.yes:
            print("当前只是 dry-run。确认轨迹安全后追加 --yes 才会执行。")
            return

        print("开始执行打靶动作。")
        controller.execute_trajectory(result.trajectory)
        print("打靶动作完成。")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
