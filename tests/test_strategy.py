"""井字棋策略模块测试。"""

import pytest

from tictactoe.strategy import (
    BoardError,
    HUMAN,
    ROBOT,
    apply_move,
    best_move,
    check_winner,
    format_board,
    is_draw,
    is_forced_draw_by_no_potential,
    legal_moves,
    normalize_board,
    outcome_probabilities,
    validate_human_move,
)


def test_normalize_board_accepts_common_tokens():
    board = normalize_board(["human", "robot", "empty", "X", "O", ".", None, "-", "_"])

    assert board == ("X", "O", ".", "X", "O", ".", ".", ".", ".")


def test_check_winner_rows_columns_and_diagonals():
    assert check_winner("XXX......") == HUMAN
    assert check_winner("O..O..O..") == ROBOT
    assert check_winner("X...X...X") == HUMAN


def test_legal_moves_returns_empty_after_win():
    assert legal_moves("XXX.O.O..") == []


def test_apply_move_does_not_modify_original_board():
    board = normalize_board("XOX......")
    moved = apply_move(board, 4, ROBOT)

    assert board == ("X", "O", "X", ".", ".", ".", ".", ".", ".")
    assert moved == ("X", "O", "X", ".", "O", ".", ".", ".", ".")


def test_validate_human_move():
    assert validate_human_move("XO.......", 2) is True
    assert validate_human_move("XO.......", 0) is False
    assert validate_human_move("XO.......", 9) is False


def test_best_move_wins_immediately():
    decision = best_move("XX.OO.X..")

    assert decision.index == 5
    assert decision.reason == "win_now"


def test_best_move_blocks_human_win():
    decision = best_move("XX..O....")

    assert decision.index == 2
    assert decision.reason == "block_human"


def test_best_move_prefers_center_on_empty_board():
    decision = best_move(".........")

    assert decision.index == 4
    assert decision.score >= 0


def test_draw_detection():
    board = "XOXOOXXXO"

    assert is_draw(board) is True
    assert best_move(board).index is None
    assert best_move(board).reason == "draw"


def test_outcome_probabilities_sum_to_one():
    probabilities = outcome_probabilities("X........")

    total = probabilities.human_win + probabilities.robot_win + probabilities.draw
    assert total == pytest.approx(1.0)
    assert probabilities.policy == "both_players_uniform_random"


def test_forced_draw_detects_no_remaining_three_in_a_row():
    assert is_forced_draw_by_no_potential("XXOOXX.OO") is True

    probabilities = outcome_probabilities("XXOOXX.OO")
    assert probabilities.draw == pytest.approx(1.0)
    assert probabilities.human_win == pytest.approx(0.0)
    assert probabilities.robot_win == pytest.approx(0.0)


def test_not_forced_draw_when_a_player_can_still_make_three():
    assert is_forced_draw_by_no_potential("X........") is False


def test_invalid_board_piece_count():
    with pytest.raises(BoardError):
        normalize_board("XXX......")


def test_format_board():
    text = format_board("XO.......")

    assert text.splitlines() == ["X O .", ". . .", ". . ."]
