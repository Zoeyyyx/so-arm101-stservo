"""井字棋视觉接口的基础测试。

这些测试不打开摄像头，也不要求安装 OpenCV。
"""

from tictactoe.strategy import EMPTY, HUMAN, ROBOT
from tictactoe.vision import (
    UNKNOWN,
    BoardStabilityFilter,
    BoardVisionConfig,
    cell_center,
    classify_cell_from_scores,
    format_vision_board,
    load_simple_yaml,
    parse_hsv_ranges,
)


def test_config_from_dict_parses_corners_and_hsv():
    config = BoardVisionConfig.from_dict(
        {
            "camera_index": 1,
            "warp_size": 300,
            "board_corners": [[10, 20], [100, 20], [100, 110], [10, 110]],
            "human_hsv_lower": [1, 2, 3],
            "human_hsv_upper": [4, 5, 6],
        }
    )

    assert config.camera_index == 1
    assert config.warp_size == 300
    assert config.has_board_corners is True
    assert config.board_corners[0] == (10.0, 20.0)
    assert config.human_hsv_lower == (1, 2, 3)
    assert config.human_hsv_upper == (4, 5, 6)


def test_config_from_dict_parses_multi_hsv_ranges():
    config = BoardVisionConfig.from_dict(
        {
            "human_hsv_ranges": [
                [[0, 90, 80], [10, 255, 255]],
                [[170, 90, 80], [179, 255, 255]],
            ],
            "robot_hsv_ranges": [
                [[0, 0, 0], [179, 255, 70]],
            ],
        }
    )

    assert config.effective_human_hsv_ranges() == (
        ((0, 90, 80), (10, 255, 255)),
        ((170, 90, 80), (179, 255, 255)),
    )
    assert config.effective_robot_hsv_ranges() == (
        ((0, 0, 0), (179, 255, 70)),
    )


def test_parse_hsv_ranges_falls_back_to_legacy_fields():
    ranges = parse_hsv_ranges(None, [0, 80, 80], [10, 255, 255])

    assert ranges == (((0, 80, 80), (10, 255, 255)),)


def test_cell_center_uses_3x3_order():
    assert cell_center(0, 600) == (100.0, 100.0)
    assert cell_center(4, 600) == (300.0, 300.0)
    assert cell_center(8, 600) == (500.0, 500.0)


def test_classify_cell_from_scores():
    assert classify_cell_from_scores(10, 5, min_blob_area=300) == EMPTY
    assert classify_cell_from_scores(500, 20, min_blob_area=300) == HUMAN
    assert classify_cell_from_scores(20, 500, min_blob_area=300) == ROBOT
    assert classify_cell_from_scores(500, 450, min_blob_area=300) == UNKNOWN


def test_stability_filter_requires_consecutive_frames():
    stable = BoardStabilityFilter(required_count=3)
    board = tuple("X.O......")

    assert stable.update(board) is None
    assert stable.update(board) is None
    assert stable.update(board) == board
    assert stable.update(tuple("XOO......")) == board
    assert stable.update(tuple("XOO......")) == board
    assert stable.update(tuple("XOO......")) == tuple("XOO......")


def test_format_vision_board_allows_unknown():
    assert format_vision_board(tuple("XO?......")) == "X O ?\n. . .\n. . ."


def test_load_simple_yaml_template_subset():
    data = load_simple_yaml(
        """
        camera_index: 0
        warp_size: 600
        board_corners:
          - null
          - [1, 2]
        human_hsv_lower: [0, 80, 80]
        human_hsv_ranges:
          - [[0, 90, 80], [10, 255, 255]]
          - [[170, 90, 80], [179, 255, 255]]
        ambiguous_ratio: 1.25
        """
    )

    assert data["camera_index"] == 0
    assert data["warp_size"] == 600
    assert data["board_corners"] == [None, [1, 2]]
    assert data["human_hsv_lower"] == [0, 80, 80]
    assert data["human_hsv_ranges"] == [
        [[0, 90, 80], [10, 255, 255]],
        [[170, 90, 80], [179, 255, 255]],
    ]
    assert data["ambiguous_ratio"] == 1.25
