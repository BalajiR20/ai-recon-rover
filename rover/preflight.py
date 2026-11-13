"""Automated preflight checks for hardware, camera, YOLO model, and arena."""
from __future__ import annotations

import time
from rover.config import AppConfig, CalibrationData
from rover.detector import PersonDetector
from rover.frames import FrameSource
from rover.grid import ArenaGrid
from rover.models import PreflightCheckItem, PreflightResult
from rover.robot import RobotInterface


def run_preflight_checks(
    cfg: AppConfig,
    cal: CalibrationData,
    grid: ArenaGrid,
    robot: RobotInterface,
    frame_source: FrameSource,
    detector: PersonDetector,
) -> PreflightResult:
    checks = []

    # 1. Arena geometry
    geom_ok = grid.cols >= 1 and grid.rows >= 1 and grid.in_bounds(cfg.arena.home_col, cfg.arena.home_row)
    checks.append(PreflightCheckItem(
        name="Arena Geometry",
        passed=geom_ok,
        detail=f"Arena {cfg.arena.width_m}x{cfg.arena.height_m}m ({grid.cols}x{grid.rows} cells of {cfg.arena.cell_size_m}m). Home: ({cfg.arena.home_col},{cfg.arena.home_row})",
    ))

    # 2. Calibration Profile
    cal_ok = cal.cm_per_s > 5.0 and cal.deg_per_s > 10.0
    checks.append(PreflightCheckItem(
        name="Calibration Parameters",
        passed=cal_ok,
        detail=f"Speed: {cal.cm_per_s:.1f} cm/s, Turn: {cal.deg_per_s:.1f} deg/s, StopDist: {cal.stop_distance_cm:.1f} cm",
    ))

    # 3. Robot Controller Link
    bot_ok = False
    try:
        st = robot.get_status()
        bot_ok = st.connected
        bot_detail = f"Connected (boot_id={st.boot_id}, ultrasonic={st.ultrasonic_cm:.1f} cm, state={st.state})"
    except Exception as exc:
        bot_detail = f"Comms error: {exc}"
    checks.append(PreflightCheckItem(
        name="ESP32 Motor/Sensor Controller",
        passed=bot_ok,
        detail=bot_detail,
        fix_instruction="Verify ESP32 is powered via USB, connected to 2.4GHz Wi-Fi, and devkit_url matches in config.yaml.",
    ))

    # 4. Camera Stream Link
    frame_ok = frame_source.is_healthy(cfg.camera.max_frame_age_s)
    checks.append(PreflightCheckItem(
        name="Camera Video Stream",
        passed=frame_ok,
        detail=f"Frame age: {frame_source.frame_age_s():.2f}s (FPS: {getattr(frame_source, 'fps', 0.0):.1f})",
        fix_instruction="Open DroidCam/IP Webcam on phone, verify IP address, and confirm stream opens in laptop browser.",
    ))

    # 5. AI Person Detector
    det_ok = detector.is_loaded
    checks.append(PreflightCheckItem(
        name="YOLO AI Vision Pipeline",
        passed=det_ok,
        detail=f"Detector loaded: device={detector.device_name}, imgsz={cfg.vision.imgsz}",
        fix_instruction="Run: pip install ultralytics and verify yolov8n.pt weights in models/ folder.",
    ))

    # 6. Grid Reachability
    reach = grid.update_reachability((cfg.arena.home_col, cfg.arena.home_row))
    reach_ok = len(reach) >= 1
    checks.append(PreflightCheckItem(
        name="Grid Reachability & Coverage Path",
        passed=reach_ok,
        detail=f"{len(reach)} reachable cells from Home ({cfg.arena.home_col},{cfg.arena.home_row})",
    ))

    passed_all = all(c.passed for c in checks)
    return PreflightResult(passed=passed_all, checks=checks, timestamp=time.time())
