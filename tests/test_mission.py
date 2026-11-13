"""End-to-end unit and integration tests for MissionController state machine, coverage completion, Smart AI Target-Confirmation, and FastAPI server."""
from __future__ import annotations

import pathlib
import pytest
from fastapi.testclient import TestClient

from rover.config import AppConfig, ArenaConfig, CalibrationData, PlannerConfig
from rover.detector import SyntheticPersonDetector
from rover.frames import SyntheticFrameSource
from rover.mission import InvalidTransitionError, MissionController
from rover.models import DetectionBox, EvidenceMode, MissionState
from rover.robot import Esp32Robot
from rover.server import create_app
from rover.store import MissionStore
from tools.mock_esp32 import MockEsp32Server


def test_state_transition_table_enforces_valid_transitions(tmp_path: pathlib.Path) -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=25.0)
    srv.start()
    cfg = AppConfig()
    cal = CalibrationData()
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    store = MissionStore(runs_root=tmp_path)
    mc = MissionController(
        cfg, cal, robot, SyntheticFrameSource(), SyntheticPersonDetector(), store, fast_scan_timing=True
    )
    try:
        with pytest.raises(InvalidTransitionError):
            mc.transition_to(MissionState.SCANNING, "invalid_jump")

        robot.configure(cal)
        st = mc.manual_move("forward", 15.0)
        assert st.state == "done"

        mc.transition_to(MissionState.PREFLIGHT)
        mc.transition_to(MissionState.PLANNING)
        mc.transition_to(MissionState.MOVING_TO_SCAN_POINT)
        with pytest.raises(InvalidTransitionError):
            mc.manual_move("forward", 10.0)
    finally:
        robot.close()
        srv.stop()


def test_full_arena_coverage_and_smart_ai_confirmation(tmp_path: pathlib.Path) -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=40.0)
    srv.start()
    cfg = AppConfig(
        arena=ArenaConfig(width_m=0.90, height_m=0.60, cell_size_m=0.30, home_col=0, home_row=0),
        planner=PlannerConfig(coverage_mode="serpentine", scan_spacing=1, max_coalesced_cells=1, smart_confirm_enabled=True),
    )
    cal = CalibrationData(cm_per_s=30.0, deg_per_s=180.0)
    robot = Esp32Robot(srv.base_url, start_background_worker=False)

    scan_call_count = {"n": 0}

    def scripted_detect(frame_index: int):
        if frame_index == 0:
            scan_call_count["n"] += 1
        n = scan_call_count["n"]
        if n == 2 and frame_index in (0, 1):
            return [DetectionBox(x1=290, y1=180, x2=350, y2=260, confidence=0.28, frame_index=frame_index)]
        if n == 3 and frame_index in (0, 1):
            return [DetectionBox(x1=270, y1=140, x2=370, y2=320, confidence=0.84, frame_index=frame_index)]
        return []

    detector = SyntheticPersonDetector(scripted_fn=scripted_detect)
    store = MissionStore(runs_root=tmp_path)
    mc = MissionController(
        cfg,
        cal,
        robot,
        SyntheticFrameSource(),
        detector,
        store,
        evidence_mode=EvidenceMode.MOCK,
        fast_scan_timing=True,
    )
    try:
        preflight = mc.start_mission(background=False)
        assert preflight.passed is True
        assert mc.state == MissionState.MISSION_COMPLETE
        assert mc.smart_confirmations_triggered >= 1
        assert len(mc.associator.targets) == 1
        assert mc.associator.targets[0].target_id == "T01"

        rep = mc.build_report()
        assert rep.coverage.visited_coverage_pct == pytest.approx(100.0)
        assert rep.coverage.observed_coverage_pct > 0.0
        assert rep.home_reached_estimated is True
        assert (store.run_dir / "report.json").exists()
        assert (store.run_dir / "coverage_map.json").exists()
        assert (store.run_dir / "events.jsonl").exists()
    finally:
        robot.close()
        srv.stop()


def test_fastapi_server_routes_and_dashboard(tmp_path: pathlib.Path) -> None:
    srv = MockEsp32Server(host="127.0.0.1", port=0, speed_scale=40.0)
    srv.start()
    cfg = AppConfig()
    cal = CalibrationData()
    robot = Esp32Robot(srv.base_url, start_background_worker=False)
    robot.configure(cal)
    store = MissionStore(runs_root=tmp_path)
    mc = MissionController(
        cfg, cal, robot, SyntheticFrameSource(), SyntheticPersonDetector(), store, fast_scan_timing=True
    )
    cfg_file = tmp_path / "config.yaml"
    cal_file = tmp_path / "calibration.json"
    app = create_app(mc, dashboard_path="dashboard/index.html", config_path=cfg_file, calibration_path=cal_file)
    client = TestClient(app)
    try:
        r_idx = client.get("/")
        assert r_idx.status_code == 200
        assert "AI-Assisted Search & Reconnaissance Rover" in r_idx.text

        r_state = client.get("/api/state")
        assert r_state.status_code == 200
        assert r_state.json()["state"] == "IDLE"

        r_pf = client.post("/api/preflight")
        assert r_pf.status_code == 200
        assert r_pf.json()["passed"] is True

        r_obs = client.post("/api/setup/obstacle", json={"col": 2, "row": 2, "blocked": True})
        assert r_obs.status_code == 200
        assert [2, 2] in r_obs.json()["grid"]["configured_obstacles"]

        r_jog = client.post("/api/command", json={"action": "manual_move", "move_type": "forward", "value": 15.0})
        assert r_jog.status_code == 200
        assert r_jog.json()["ok"] is True

        r_home = client.post("/api/command", json={"action": "verify_home", "verified": True, "offset_cm": 6.5})
        assert r_home.status_code == 200
    finally:
        robot.close()
        srv.stop()
