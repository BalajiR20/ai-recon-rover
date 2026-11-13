"""Unit tests for blocking preflight verification checks and fix instructions."""
from __future__ import annotations

import requests
from rover.config import AppConfig, CalibrationData
from rover.grid import ArenaGrid
from rover.preflight import run_preflight_checks
from rover.robot import Esp32Robot
from tools.mock_esp32 import MockEsp32Server


class _DummyFrameHealth:
    def __init__(self, healthy: bool = True, age: float = 0.08) -> None:
        self._healthy = healthy
        self._age = age

    def is_healthy(self, max_age_s: float) -> bool:
        return self._healthy and self._age <= max_age_s

    def frame_age_s(self) -> float:
        return self._age


class _DummyDetectorHealth:
    def __init__(self, loaded: bool = True, device: str = "CPU") -> None:
        self.is_loaded = loaded
        self.device_name = device


def test_preflight_all_pass_and_failure_fixes() -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=10.0)
    srv.start()
    cfg = AppConfig()
    cal = CalibrationData()
    grid = ArenaGrid(cfg.arena, cfg.planner, cfg.camera)
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    try:
        res = run_preflight_checks(
            cfg,
            cal,
            grid,
            robot,
            frame_source=_DummyFrameHealth(True, 0.05),
            detector=_DummyDetectorHealth(True, "CPU"),
        )
        assert res.passed is True
        assert len(res.checks) == 6

        # Now inject invalid ultrasonic reading (-1.0) and latched E-stop
        requests.post(f"{srv.base_url}/mock/inject", json={"forced_ultrasonic_cm": -1.0}, timeout=1.0)
        robot.estop()
        res_fail = run_preflight_checks(
            cfg,
            cal,
            grid,
            robot,
            frame_source=_DummyFrameHealth(False, 3.2),
            detector=_DummyDetectorHealth(True, "CPU"),
        )
        assert res_fail.passed is False
        by_name = {c.name: c for c in res_fail.checks}
        assert by_name["devkit_protocol_and_state"].passed is False
        assert "Reset E-Stop" in by_name["devkit_protocol_and_state"].fix_instruction
        assert by_name["camera_and_detector"].passed is False
        assert by_name["ultrasonic_clearance"].passed is False
        assert "1k/2k" in by_name["ultrasonic_clearance"].fix_instruction
    finally:
        robot.close()
        srv.stop()
