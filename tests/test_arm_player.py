"""井字棋机械臂落点适配层测试。"""

import pytest

from tictactoe.arm_player import (
    ArmCellPlayer,
    ArmConfigError,
    CellTarget,
    TictactoeArmConfig,
    generate_grid_cells,
    load_simple_nested_yaml,
    render_arm_config_yaml,
)


def make_config_with_center() -> TictactoeArmConfig:
    config = TictactoeArmConfig.empty()
    config.cells[4] = CellTarget(
        index=4,
        x=0.20,
        y=0.01,
        z_above=0.05,
        z_press=-0.10,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        label="center",
    )
    return config


def test_cell_target_requires_basic_coordinates():
    cell = CellTarget(index=0, x=0.1, y=None, z_press=-0.1)

    assert cell.is_configured() is False
    assert cell.missing_fields() == ["y"]


def test_cell_target_cli_args():
    cell = CellTarget(index=4, x=0.2, y=0.0, z_above=0.03, z_press=-0.09)
    args = cell.cli_target_args("so101_base", default_strike_height=0.12)

    assert "--x" in args
    assert "0.200000" in args
    assert "--z" in args
    assert "-0.090000" in args
    assert "--strike-height" in args
    assert "0.120000" in args


def test_cell_target_rejects_bad_height():
    cell = CellTarget(index=4, x=0.2, y=0.0, z_above=-0.2, z_press=-0.1)

    with pytest.raises(ArmConfigError):
        cell.cli_target_args("so101_base", default_strike_height=0.12)


def test_generate_grid_cells_row_major_order():
    cells = generate_grid_cells(
        origin_x=0.10,
        origin_y=-0.06,
        col_dx=0.03,
        col_dy=0.00,
        row_dx=0.00,
        row_dy=0.03,
        z_press=-0.12,
        z_above=0.02,
    )

    assert cells[0].x == pytest.approx(0.10)
    assert cells[0].y == pytest.approx(-0.06)
    assert cells[1].x == pytest.approx(0.13)
    assert cells[3].y == pytest.approx(-0.03)
    assert cells[8].x == pytest.approx(0.16)
    assert cells[8].y == pytest.approx(0.00)


def test_config_reports_missing_and_configured_cells():
    config = make_config_with_center()

    assert config.configured_cells() == [4]
    assert 4 not in config.missing_cells()
    assert 0 in config.missing_cells()


def test_build_hit_command_for_cell():
    config = make_config_with_center()
    player = ArmCellPlayer(config, repo_root="C:/project")

    command = player.build_hit_command(4, execute=True, port="COM9")

    assert "hit_target_action.py" in " ".join(command)
    assert "--port" in command
    assert "COM9" in command
    assert "--yes" in command
    assert "0.200000" in command


def test_build_hit_command_rejects_unconfigured_cell():
    config = TictactoeArmConfig.empty()
    player = ArmCellPlayer(config, repo_root="C:/project")

    with pytest.raises(ArmConfigError):
        player.build_hit_command(0)


def test_simple_yaml_loader_reads_nested_config():
    data = load_simple_nested_yaml(
        """
        robot:
          port: COM5
          baudrate: 1000000
        cells:
          4:
            x: 0.2
            y: 0.0
            z_press: -0.1
        """
    )
    config = TictactoeArmConfig.from_dict(data)

    assert config.port == "COM5"
    assert config.baudrate == 1000000
    assert config.cells[4].x == pytest.approx(0.2)
    assert config.cells[4].z_press == pytest.approx(-0.1)


def test_render_arm_config_yaml_can_round_trip_with_simple_loader():
    config = make_config_with_center()
    text = render_arm_config_yaml(config)
    loaded = TictactoeArmConfig.from_dict(load_simple_nested_yaml(text))

    assert loaded.cells[4].x == pytest.approx(0.20)
    assert loaded.cells[4].z_above == pytest.approx(0.05)
    assert loaded.cells[4].label == "center"
