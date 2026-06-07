"""井字棋主流程管理测试。"""

from dataclasses import dataclass

from tictactoe.arm_player import ArmCellPlayer, CellTarget, TictactoeArmConfig
from tictactoe.game_manager import TicTacToeGameManager, normalize_observed_board, result_summary_lines
from tictactoe.strategy import ROBOT
from scripts.run_tictactoe_game import board_changed_enough


@dataclass
class FakeArmPlan:
    success: bool = True
    reason: str = ""
    trajectory: list = None


class FakeArmPlayer:
    def __init__(self):
        self.planned = []

    def build_hit_command(self, index, *, execute=False, port=None):
        command = ["python", "hit_target_action.py", "--cell", str(index)]
        if execute:
            command.append("--yes")
        if port:
            command.extend(["--port", port])
        return command

    def plan_or_execute_cell(self, index, *, execute=False, port=None):
        self.planned.append((index, execute, port))
        return FakeArmPlan(success=True, trajectory=[]), object(), object()


def test_normalize_observed_board_allows_unknown():
    assert normalize_observed_board("XO?......") == ("X", "O", "?", ".", ".", ".", ".", ".", ".")


def test_empty_board_waits_for_human():
    manager = TicTacToeGameManager()
    result = manager.process_board(".........")

    assert result.success is True
    assert result.status == "waiting_for_human"


def test_unknown_board_is_rejected():
    manager = TicTacToeGameManager()
    result = manager.process_board("X?.......")

    assert result.success is False
    assert result.status == "vision_unknown"


def test_human_win_finishes_game():
    manager = TicTacToeGameManager()
    result = manager.process_board("XXXOO....")

    assert result.success is True
    assert result.status == "human_won"
    assert result.winner == "X"


def test_forced_draw_finishes_game_early():
    manager = TicTacToeGameManager()
    result = manager.process_board("XXOOXX.OO")

    assert result.success is True
    assert result.status == "forced_draw"
    assert result.probabilities.draw == 1.0
    assert result.robot_cell is None


def test_robot_decision_without_arm_player():
    manager = TicTacToeGameManager()
    result = manager.process_board("X........")

    assert result.success is True
    assert result.status == "robot_move_ready"
    assert result.robot_cell == 4
    assert result.after_board[4] == ROBOT


def test_unconfigured_real_arm_config_does_not_block_decision_preview():
    config = TictactoeArmConfig.empty()
    player = ArmCellPlayer(config, repo_root="C:/project")
    manager = TicTacToeGameManager(player)

    result = manager.process_board("X........")

    assert result.success is True
    assert result.status == "robot_move_needs_arm_calibration"
    assert result.robot_cell == 4
    assert "尚未完整标定" in result.reason


def test_fake_arm_player_builds_command_and_plans():
    player = FakeArmPlayer()
    manager = TicTacToeGameManager(player)

    result = manager.process_board("X........", plan=True, port="COM9")

    assert result.success is True
    assert result.status == "robot_move_ready"
    assert result.command == ["python", "hit_target_action.py", "--cell", "4", "--port", "COM9"]
    assert result.planned is True
    assert player.planned == [(4, False, "COM9")]


def test_fake_arm_player_execute_flag():
    player = FakeArmPlayer()
    manager = TicTacToeGameManager(player)

    result = manager.process_board("X........", execute=True)

    assert result.success is True
    assert result.executed is True
    assert "--yes" in result.command
    assert player.planned == [(4, True, None)]


def test_result_summary_contains_expected_board():
    manager = TicTacToeGameManager(FakeArmPlayer())
    result = manager.process_board("X........")
    text = "\n".join(result_summary_lines(result))

    assert "当前棋盘" in text
    assert "当前胜负概率" in text
    assert "落子后胜负概率" in text
    assert "机械臂决策: cell=4" in text
    assert "机械臂落子后的预期棋盘" in text


def test_configured_real_arm_player_can_build_command_in_manager():
    config = TictactoeArmConfig.empty()
    config.cells[4] = CellTarget(index=4, x=0.2, y=0.0, z_press=-0.1, z_above=0.02)
    player = ArmCellPlayer(config, repo_root="C:/project")
    manager = TicTacToeGameManager(player)

    result = manager.process_board("X........")

    assert result.success is True
    assert result.status == "robot_move_ready"
    assert "--strike-height" in result.command


def test_live_board_change_filter():
    board = tuple("X........")

    assert board_changed_enough(None, board) is True
    assert board_changed_enough(board, board) is False
    assert board_changed_enough(board, tuple("XO.......")) is True
