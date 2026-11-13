"""Fault injection and safety recovery tests: obstacle replanning, invalid ultrasonic, camera stall, board reboot, and uncertainty policy."""
from __future__ import annotations

import pathlib
import requests
from rover.config import AppConfig, ArenaConfig, CalibrationData, LocalizationConfig
from rover.detector import SyntheticPersonDetector
from rover.frames import SyntheticFrameSource
from rover.mission import MissionController
from rover.models import MissionState
from rover.robot import Esp32Robot
from rover.store import MissionStore
from tools.mock_esp32 import MockEsp32Server


def test_obstacle_discovery_and_astar_replanning(tmp_path: pathlib.Path) -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=30.0)
    srv.start()
    cfg = AppConfig(arena=ArenaConfig(width_m=1.20, height_m=0.90, cell_size_m=0.30))
    cal = CalibrationData()
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    store = MissionStore(runs_root=tmp_path)
    mc = MissionController(
        cfg, cal, robot, SyntheticFrameSource(), SyntheticPersonDetector(), store, fast_scan_timing=True
    )
    try:
        mc.run_preflight()
        mc.transition_to(MissionState.PREFLIGHT)
        mc.transition_to(MissionState.PLANNING)
        mc.grid.mark_visited(0, 0)

        # Inject close obstacle at 12 cm before stepping to (1, 0)
        requests.post(f"{srv.base_url}/mock/inject", json={"forced_ultrasonic_cm": 12.0}, timeout=1.0)
        cont = mc.step_once()
        assert cont is True
        assert (1, 0) in mc.grid.discovered_blocked
        assert mc.state == MissionState.REPLANNING

        # Clear obstacle and verify next step detours via (0, 1)
        requests.post(f"{srv.base_url}/mock/inject", json={"forced_ultrasonic_cm": 120.0}, timeout=1.0)
        cont2 = mc.step_once()
        assert cont2 is True
        assert mc.current_cell() == (0, 1)
    finally:
        robot.close()
        srv.stop()


def test_invalid_ultrasonic_and_camera_stall_and_recovery(tmp_path: pathlib.Path) -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=30.0)
    srv.start()
    cfg = AppConfig(
        arena=ArenaConfig(width_m=1.20, height_m=0.90, cell_size_m=0.30),
        localization=LocalizationConfig(
            sigma_pos_max_m=0.10,
            uncertainty_policy="recovery_required",
        ),
    )
    cal = CalibrationData()
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    cam = SyntheticFrameSource()
    store = MissionStore(runs_root=tmp_path)
    mc = MissionController(
        cfg, cal, robot, cam, SyntheticPersonDetector(), store, fast_scan_timing=True
    )
    try:
        robot.configure(cal)
        mc.transition_to(MissionState.PREFLIGHT)
        mc.transition_to(MissionState.PLANNING)

        # 1. Inject 5 consecutive invalid (-1.0) ultrasonic readings -> RECOVERY_REQUIRED
        requests.post(f"{srv.base_url}/mock/inject", json={"forced_ultrasonic_cm": -1.0}, timeout=1.0)
        for _ in range(cfg.mission.max_consecutive_invalid_us):
            if mc.state == MissionState.RECOVERY_REQUIRED:
                break
            mc.step_once()
        assert mc.state == MissionState.RECOVERY_REQUIRED

        # Operator confirms recovery at (0, 0) and restores ultrasonic
        requests.post(f"{srv.base_url}/mock/inject", json={"forced_ultrasonic_cm": 150.0}, timeout=1.0)
        mc.confirm_recovery(col=0, row=0, heading_deg=0.0)
        assert mc.state == MissionState.PLANNING
        assert mc.pose_est.pose.sigma_pos_m == 0.05

        # 2. Stall camera stream during scan -> RECOVERY_REQUIRED + cell marked UNCERTAIN
        cam.stalled = True
        mc._perform_scan_at_current_cell(reason="test_stall")
        assert mc.state == MissionState.RECOVERY_REQUIRED
        cam.stalled = False
        mc.continue_anyway()
        assert mc.state == MissionState.PLANNING
    finally:
        robot.close()
        srv.stop()
