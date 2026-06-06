"""井字棋主流程管理。

本模块串联三件事：
1. 接收当前棋盘状态，可以来自视觉，也可以来自手动输入；
2. 调用 strategy.py 计算机械臂下一步；
3. 调用 arm_player.py 把格子编号转换成机械臂打靶动作。

默认规则：人类玩家为 X，机械臂为 O，人类先手。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

from .arm_player import ArmCellPlayer, ArmConfigError
from .strategy import (
    EMPTY,
    HUMAN,
    ROBOT,
    BoardError,
    MoveDecision,
    apply_move,
    best_move,
    check_winner,
    is_draw,
    normalize_board,
    normalize_cell,
)
from .vision import UNKNOWN, format_vision_board


@dataclass(frozen=True)
class GameTurnResult:
    """处理一帧棋盘后的结果。"""

    success: bool
    status: str
    board: tuple[str, ...]
    reason: str = ""
    winner: str | None = None
    decision: MoveDecision | None = None
    robot_cell: int | None = None
    after_board: tuple[str, ...] | None = None
    command: list[str] = field(default_factory=list)
    planned: bool = False
    executed: bool = False
    arm_plan: Any = None
    diagnostics: dict[str, Any] = field(default_factory=dict)

    @property
    def board_text(self) -> str:
        return format_vision_board(self.board)

    @property
    def after_board_text(self) -> str | None:
        if self.after_board is None:
            return None
        return format_vision_board(self.after_board)


class TicTacToeGameManager:
    """井字棋机械臂 demo 的高层调度器。"""

    def __init__(self, arm_player: ArmCellPlayer | None = None):
        self.arm_player = arm_player

    def process_board(
        self,
        board: Sequence[object] | str,
        *,
        plan: bool = False,
        execute: bool = False,
        port: str | None = None,
    ) -> GameTurnResult:
        """根据当前棋盘决定机械臂是否需要落子。

        plan=True 会连接机械臂做 dry-run 规划。
        execute=True 会在规划成功后真实发送动作。
        两者都为 False 时，只生成等价命令，不连接机械臂。
        """

        try:
            observed = normalize_observed_board(board)
        except (BoardError, ValueError) as exc:
            return GameTurnResult(False, "invalid_board", tuple(), reason=str(exc))

        if UNKNOWN in observed:
            return GameTurnResult(
                False,
                "vision_unknown",
                observed,
                reason="棋盘中存在 ?，视觉结果不确定，不能让机械臂落子。",
            )

        try:
            normalized = normalize_board(observed)
        except BoardError as exc:
            return GameTurnResult(False, "invalid_board", observed, reason=str(exc))

        winner = check_winner(normalized)
        if winner is not None:
            status = "human_won" if winner == HUMAN else "robot_won"
            return GameTurnResult(True, status, normalized, winner=winner, reason=f"{winner} 已经获胜。")

        if is_draw(normalized):
            return GameTurnResult(True, "draw", normalized, reason="棋盘已满，平局。")

        turn_status = robot_turn_status(normalized)
        if turn_status != "robot_turn":
            return GameTurnResult(True, turn_status, normalized, reason=turn_status_reason(turn_status))

        decision = best_move(normalized)
        if decision.index is None:
            return GameTurnResult(True, "no_robot_move", normalized, decision=decision, reason=decision.reason)

        after_board = apply_move(normalized, decision.index, ROBOT)
        command: list[str] = []
        arm_plan = None
        planned = False
        executed = False

        if self.arm_player is not None:
            try:
                command = self.arm_player.build_hit_command(decision.index, execute=execute, port=port)
            except ArmConfigError as exc:
                if not plan and not execute:
                    return GameTurnResult(
                        True,
                        "robot_move_needs_arm_calibration",
                        normalized,
                        reason=str(exc),
                        decision=decision,
                        robot_cell=decision.index,
                        after_board=after_board,
                    )
                return GameTurnResult(
                    False,
                    "arm_config_error",
                    normalized,
                    reason=str(exc),
                    decision=decision,
                    robot_cell=decision.index,
                    after_board=after_board,
                )

            if plan or execute:
                try:
                    arm_plan, _target_pose_base, _state = self.arm_player.plan_or_execute_cell(
                        decision.index,
                        execute=execute,
                        port=port,
                    )
                except Exception as exc:  # noqa: BLE001 - 硬件/IK错误需要原样回传给上层
                    return GameTurnResult(
                        False,
                        "arm_runtime_error",
                        normalized,
                        reason=str(exc),
                        decision=decision,
                        robot_cell=decision.index,
                        after_board=after_board,
                        command=command,
                    )
                planned = True
                if not getattr(arm_plan, "success", False):
                    return GameTurnResult(
                        False,
                        "arm_plan_failed",
                        normalized,
                        reason=getattr(arm_plan, "reason", "机械臂规划失败"),
                        decision=decision,
                        robot_cell=decision.index,
                        after_board=after_board,
                        command=command,
                        planned=True,
                        arm_plan=arm_plan,
                    )
                executed = bool(execute)

        return GameTurnResult(
            True,
            "robot_move_ready",
            normalized,
            reason=decision.reason,
            decision=decision,
            robot_cell=decision.index,
            after_board=after_board,
            command=command,
            planned=planned,
            executed=executed,
            arm_plan=arm_plan,
        )


def normalize_observed_board(board: Sequence[object] | str) -> tuple[str, ...]:
    """标准化视觉/手动输入棋盘，允许 ? 暂存。"""

    if isinstance(board, str):
        compact = board.replace(" ", "").replace("\n", "")
        cells: Sequence[object] = tuple(compact)
    else:
        cells = board

    normalized = []
    for cell in cells:
        text = str(cell).strip()
        if text == UNKNOWN or text.lower() in {"unknown", "?"}:
            normalized.append(UNKNOWN)
        else:
            normalized.append(normalize_cell(cell))

    if len(normalized) != 9:
        raise ValueError(f"棋盘必须正好包含 9 个格子，当前为 {len(normalized)} 个")
    return tuple(normalized)


def robot_turn_status(board: Sequence[str]) -> str:
    """判断当前是否轮到机械臂行动。"""

    x_count = board.count(HUMAN)
    o_count = board.count(ROBOT)
    if x_count == o_count:
        return "waiting_for_human"
    if x_count == o_count + 1:
        return "robot_turn"
    return "invalid_turn_count"


def turn_status_reason(status: str) -> str:
    if status == "waiting_for_human":
        return "当前应由人类 X 落子，机械臂等待。"
    if status == "invalid_turn_count":
        return "X/O 数量不符合人类先手规则，请检查视觉识别或棋盘状态。"
    return status


def result_summary_lines(result: GameTurnResult) -> list[str]:
    """把结构化结果转成 CLI 友好的文本。"""

    lines = [
        f"状态: {result.status}",
        "当前棋盘:",
        result.board_text if result.board else "<invalid>",
    ]
    if result.winner:
        lines.append(f"胜者: {result.winner}")
    if result.decision:
        lines.append(
            f"机械臂决策: cell={result.decision.index} "
            f"reason={result.decision.reason} score={result.decision.score}"
        )
    if result.after_board is not None:
        lines.extend(["机械臂落子后的预期棋盘:", result.after_board_text or ""])
    if result.command:
        lines.append("等价机械臂命令:")
        lines.append(" ".join(result.command))
    if result.planned:
        lines.append("机械臂规划: 已完成 dry-run 规划")
    if result.executed:
        lines.append("机械臂动作: 已执行")
    if result.reason:
        lines.append(f"说明: {result.reason}")
    return lines
