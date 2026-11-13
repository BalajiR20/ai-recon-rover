"""Integration tests for Hardware Protocol v1 between Esp32Robot client and MockEsp32Server over HTTP."""
from __future__ import annotations

import time
import pytest
import requests

from rover.config import CalibrationData, RobotConfig
from rover.robot import Esp32Robot
from tools.mock_esp32 import MockEsp32Server


@pytest.fixture
def mock_server():
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=10.0)
    srv.start()
    yield srv
    srv.stop()


def test_protocol_config_required_and_idempotency(mock_server: MockEsp32Server) -> None:
    robot = Esp32Robot(mock_server.base_url, start_background_worker=False)
    try:
        # Move before /config must be rejected with reason="not_configured"
        ack0 = robot.send_move("forward", 20.0, cmd_id=1)
        assert ack0.accepted is False
        assert ack0.reason == "not_configured"

        # Configure DevKit
        cfg_resp = robot.configure(CalibrationData())
        assert cfg_resp["accepted"] is True
        assert cfg_resp["protocol"] == "v1"

        # Send move with id=5
        ack1 = robot.send_move("forward", 30.0, cmd_id=5)
        assert ack1.accepted is True
        assert ack1.duplicate is False

        # Resending same id=5 must be idempotent (accepted=True, duplicate=True)
        ack_dup = robot.send_move("forward", 30.0, cmd_id=5)
        assert ack_dup.accepted is True
        assert ack_dup.duplicate is True

        # Sending stale id=3 must be rejected with reason="stale_id"
        ack_stale = robot.send_move("forward", 15.0, cmd_id=3)
        assert ack_stale.accepted is False
        assert ack_stale.reason == "stale_id"

        st = robot.wait_for_move(timeout_s=3.0)
        assert st.state == "done"
        assert st.executed_cm == pytest.approx(30.0, abs=1.0)
    finally:
        robot.close()


def test_obstacle_abort_and_partial_executed_cm(mock_server: MockEsp32Server) -> None:
    mock_server.hw.speed_scale = 2.0  # 50 cm/s so 30 cm takes 0.6s
    robot = Esp32Robot(mock_server.base_url, start_background_worker=False)
    try:
        robot.configure(CalibrationData(cm_per_s=25.0, stop_distance_cm=20.0))
        ack = robot.send_move("forward", 30.0)
        assert ack.accepted is True
        time.sleep(0.15)
        # Inject obstacle at 15 cm mid-move
        requests.post(f"{mock_server.base_url}/mock/inject", json={"forced_ultrasonic_cm": 15.0}, timeout=1.0)
        st = robot.wait_for_move(timeout_s=2.0)
        assert st.state == "aborted_obstacle"
        assert 0.0 < st.executed_cm < 30.0
    finally:
        robot.close()


def test_watchdog_abort_and_estop_latch(mock_server: MockEsp32Server) -> None:
    mock_server.hw.speed_scale = 1.0  # 25 cm/s -> 50 cm takes 2.0s
    robot = Esp32Robot(mock_server.base_url, start_background_worker=False)
    try:
        robot.configure(CalibrationData(cm_per_s=25.0, watchdog_ms=300))
        ack = robot.send_move("forward", 50.0)
        assert ack.accepted is True
        # Do not poll for 450 ms (> 300 ms watchdog_ms)
        time.sleep(0.45)
        st = robot.get_status()
        assert st.state == "aborted_watchdog"
        assert 0.0 < st.executed_cm < 50.0

        # Test E-stop latch and reset
        st_estop = robot.estop()
        assert st_estop.state == "estopped"
        ack_blocked = robot.send_move("turn", 90.0)
        assert ack_blocked.accepted is False
        assert ack_blocked.reason == "estopped"

        st_reset = robot.reset_estop()
        assert st_reset.state == "idle"
    finally:
        robot.close()


def test_boot_id_reboot_detection(mock_server: MockEsp32Server) -> None:
    reboot_events = []
    robot = Esp32Robot(
        mock_server.base_url,
        start_background_worker=False,
        on_reboot_detected=lambda old_id, new_id: reboot_events.append((old_id, new_id)),
    )
    try:
        robot.configure(CalibrationData())
        requests.post(f"{mock_server.base_url}/mock/inject", json={"reboot": True}, timeout=1.0)
        robot.get_status()
        assert robot.reboot_detected is True
        assert len(reboot_events) == 1
    finally:
        robot.close()
