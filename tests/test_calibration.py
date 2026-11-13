"""Unit tests for calibration formulas, hardware test step runner, and persistence."""
from __future__ import annotations

import pathlib
import pytest

from rover.calibrate import (
    apply_straight_calibration,
    apply_turn_calibration,
    compute_cm_per_s,
    compute_deg_per_s,
    compute_wheel_trims,
    run_calibration_test_step,
    run_interactive_cli_calibration,
)
from rover.config import CalibrationData, load_calibration, save_calibration, save_config, AppConfig
from rover.robot import Esp32Robot
from tools.mock_esp32 import MockEsp32Server


def test_calibration_math_and_persistence(tmp_path: pathlib.Path) -> None:
    cal = CalibrationData()
    apply_straight_calibration(cal, measured_distance_cm=84.0, test_duration_s=3.0, lateral_drift_cm=4.2)
    assert cal.cm_per_s == pytest.approx(28.0)
    assert cal.left_trim < 1.0
    assert cal.right_trim == pytest.approx(1.0)

    apply_turn_calibration(cal, actual_angle_deg=100.0, commanded_angle_deg=90.0)
    # 90 deg at 90 deg/s took 1.0 s -> actual 100 deg in 1.0 s = 100 deg/s
    assert cal.deg_per_s == pytest.approx(100.0)

    cal_file = tmp_path / "calibration.json"
    save_calibration(cal, cal_file)
    loaded = load_calibration(cal_file)
    assert loaded.cm_per_s == pytest.approx(28.0)
    assert loaded.deg_per_s == pytest.approx(100.0)


def test_interactive_cli_calibration_and_steps(tmp_path: pathlib.Path) -> None:
    cfg_file = tmp_path / "config.yaml"
    cal_file = tmp_path / "calibration.json"
    save_config(AppConfig(), cfg_file)
    save_calibration(CalibrationData(), cal_file)

    inputs = iter(["custom", "y", "n", "75.0", "0.0", "90.0"])
    cal = run_interactive_cli_calibration(cfg_file, cal_file, input_fn=lambda _p: next(inputs))
    assert cal.motor_invert_left is True
    assert cal.motor_invert_right is False
    assert cal.cm_per_s == pytest.approx(25.0)
    assert cal.deg_per_s == pytest.approx(90.0)

    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=15.0)
    srv.start()
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    try:
        res_ping = run_calibration_test_step(robot, cal, "ping")
        assert res_ping["ok"] is True
        res_us = run_calibration_test_step(robot, cal, "ultrasonic_test")
        assert res_us["ok"] is True
    finally:
        robot.close()
        srv.stop()
