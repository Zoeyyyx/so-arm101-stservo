"""井字棋策略模块。

本模块只处理 3x3 棋盘逻辑，不关心摄像头、机械臂或串口。
约定棋盘一共有 9 个格子，编号如下：

    0 | 1 | 2
    --+---+--
    3 | 4 | 5
    --+---+--
    6 | 7 | 8

格子状态约定：
- "X"：人类玩家
- "O"：机械臂
- "."：空格
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence


HUMAN = "X"
ROBOT = "O"
EMPTY = "."

WIN_LINES = (
    (0, 1, 2),
    (3, 4, 5),
    (6, 7, 8),
    (0, 3, 6),
    (1, 4, 7),
    (2, 5, 8),
    (0, 4, 8),
    (2, 4, 6),
)

# 同分时优先下中心、角、边，动作更像正常井字棋玩家。
PREFERRED_MOVES = (4, 0, 2, 6, 8, 1, 3, 5, 7)


class BoardError(ValueError):
    """棋盘输入不合法时抛出。"""


@dataclass(frozen=True)
class MoveDecision:
    """机械臂下一步落子决策。"""

    index: int | None
    reason: str
    score: int


def normalize_cell(cell: object) -> str:
    """把外部输入统一转换为 X/O/.。

    这样视觉模块后续可以输出 "human"、"robot"、"empty"，
    策略层仍然能稳定处理。
    """

    if cell is None:
        return EMPTY

    text = str(cell).strip()
    upper = text.upper()
    lower = text.lower()

    if text in {"", ".", "-", "_"} or lower in {"empty", "none", "blank"}:
        return EMPTY
    if upper == "X" or lower in {"human", "player", "person"}:
        return HUMAN
    if upper == "O" or lower in {"robot", "arm", "ai", "computer"}:
        return ROBOT

    raise BoardError(f"无法识别的棋子状态: {cell!r}")


def normalize_board(board: Sequence[object] | str, *, validate_turns: bool = True) -> tuple[str, ...]:
    """校验并标准化棋盘。"""

    if isinstance(board, str):
        compact = board.replace(" ", "").replace("\n", "")
        cells: Iterable[object] = compact
    else:
        cells = board

    normalized = tuple(normalize_cell(cell) for cell in cells)
    if len(normalized) != 9:
        raise BoardError(f"棋盘必须正好包含 9 个格子，当前为 {len(normalized)} 个")

    if validate_turns:
        x_count = normalized.count(HUMAN)
        o_count = normalized.count(ROBOT)
        if o_count > x_count:
            raise BoardError("机械臂 O 的数量不能多于人类 X")
        if x_count - o_count > 1:
            raise BoardError("人类 X 的数量最多只能比机械臂 O 多 1")

        winner = check_winner(normalized)
        if winner == HUMAN and x_count == o_count:
            raise BoardError("X 已获胜时，X 的数量应比 O 多 1")
        if winner == ROBOT and x_count != o_count:
            raise BoardError("O 已获胜时，X 和 O 的数量应相同")

    return normalized


def legal_moves(board: Sequence[object] | str) -> list[int]:
    """返回所有可落子的格子编号。"""

    normalized = normalize_board(board)
    if check_winner(normalized) is not None:
        return []
    return [index for index, cell in enumerate(normalized) if cell == EMPTY]


def check_winner(board: Sequence[object] | str) -> str | None:
    """判断当前棋盘是否已有胜者。"""

    # check_winner 会被 normalize_board 调用，因此这里不能反向调用完整校验。
    if isinstance(board, str):
        compact = board.replace(" ", "").replace("\n", "")
        cells = tuple(normalize_cell(cell) for cell in compact)
    else:
        cells = tuple(normalize_cell(cell) for cell in board)

    if len(cells) != 9:
        raise BoardError(f"棋盘必须正好包含 9 个格子，当前为 {len(cells)} 个")

    winners: set[str] = set()
    for a, b, c in WIN_LINES:
        if cells[a] != EMPTY and cells[a] == cells[b] == cells[c]:
            winners.add(cells[a])

    if len(winners) > 1:
        raise BoardError("棋盘同时存在两个胜者，状态不合法")
    return next(iter(winners), None)


def is_draw(board: Sequence[object] | str) -> bool:
    """判断是否平局。"""

    normalized = normalize_board(board)
    return check_winner(normalized) is None and EMPTY not in normalized


def validate_human_move(board: Sequence[object] | str, index: int) -> bool:
    """检查人类玩家是否可以在指定格子落子。"""

    normalized = normalize_board(board)
    return index in range(9) and normalized[index] == EMPTY and check_winner(normalized) is None


def apply_move(board: Sequence[object] | str, index: int, player: str) -> tuple[str, ...]:
    """返回落子后的新棋盘，不修改原始对象。"""

    # minimax 会生成很多“中间模拟棋盘”，这些棋盘可能暂时不满足外部轮次规则。
    normalized = list(normalize_board(board, validate_turns=False))
    mark = normalize_cell(player)
    if mark not in {HUMAN, ROBOT}:
        raise BoardError("落子方必须是 X 或 O")
    if index not in range(9):
        raise BoardError(f"格子编号必须是 0..8，当前为 {index}")
    if normalized[index] != EMPTY:
        raise BoardError(f"格子 {index} 已经有棋子")

    normalized[index] = mark
    return tuple(normalized)


def best_move(board: Sequence[object] | str, robot: str = ROBOT, human: str = HUMAN) -> MoveDecision:
    """计算机械臂下一步。

    策略采用完整 minimax：能赢就赢，不能赢就堵，必要时争取平局。
    井字棋状态空间很小，这种写法足够快，也方便后续调试。
    """

    normalized = normalize_board(board)
    robot_mark = normalize_cell(robot)
    human_mark = normalize_cell(human)
    if robot_mark == human_mark or EMPTY in {robot_mark, human_mark}:
        raise BoardError("robot 和 human 必须是不同的 X/O 棋子")

    winner = check_winner(normalized)
    if winner == robot_mark:
        return MoveDecision(index=None, reason="robot_already_won", score=10)
    if winner == human_mark:
        return MoveDecision(index=None, reason="human_already_won", score=-10)
    if is_draw(normalized):
        return MoveDecision(index=None, reason="draw", score=0)

    best_index: int | None = None
    best_score = -999
    for index in ordered_moves(normalized):
        candidate = apply_move(normalized, index, robot_mark)
        score = minimax(candidate, turn=human_mark, robot=robot_mark, human=human_mark, depth=1)
        if score > best_score:
            best_index = index
            best_score = score

    if best_index is None:
        return MoveDecision(index=None, reason="no_legal_move", score=0)

    return MoveDecision(index=best_index, reason=classify_reason(normalized, best_index, robot_mark, human_mark), score=best_score)


def ordered_moves(board: Sequence[str]) -> list[int]:
    """按固定优先级返回可下位置，保证策略输出稳定。"""

    empty = {index for index, cell in enumerate(board) if cell == EMPTY}
    return [index for index in PREFERRED_MOVES if index in empty]


def minimax(board: tuple[str, ...], turn: str, robot: str, human: str, depth: int) -> int:
    """井字棋 minimax 评分。"""

    winner = check_winner(board)
    if winner == robot:
        return 10 - depth
    if winner == human:
        return depth - 10
    if EMPTY not in board:
        return 0

    if turn == robot:
        return max(
            minimax(apply_move(board, index, robot), human, robot, human, depth + 1)
            for index in ordered_moves(board)
        )

    return min(
        minimax(apply_move(board, index, human), robot, robot, human, depth + 1)
        for index in ordered_moves(board)
    )


def classify_reason(board: tuple[str, ...], index: int, robot: str, human: str) -> str:
    """给调试日志一个可读原因。"""

    if check_winner(apply_move(board, index, robot)) == robot:
        return "win_now"
    if any(check_winner(apply_move(board, move, human)) == human for move in ordered_moves(board)):
        blocking_board = apply_move(board, index, robot)
        if not any(check_winner(apply_move(blocking_board, move, human)) == human for move in ordered_moves(blocking_board)):
            return "block_human"
    if index == 4:
        return "take_center"
    if index in {0, 2, 6, 8}:
        return "take_corner"
    return "best_minimax"


def format_board(board: Sequence[object] | str) -> str:
    """把棋盘格式化成适合终端打印的三行文本。"""

    cells = normalize_board(board)
    rows = [" ".join(cells[row : row + 3]) for row in range(0, 9, 3)]
    return "\n".join(rows)
