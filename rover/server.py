"""FastAPI server with MJPEG proxy, /report evaluation sheet, and WebSocket telemetry."""
from __future__ import annotations

import asyncio
import pathlib
import time
from typing import Any, Dict, Optional
import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.responses import HTMLResponse, PlainTextResponse, StreamingResponse
from pydantic import BaseModel

from rover.calibrate import apply_straight_calibration, apply_turn_calibration
from rover.config import save_calibration, save_config
from rover.detector import detect_ground_hazards
from rover.mission import MissionController
from rover.models import DetectionBox
from rover.robot import RobotCommunicationError
from rover.tracker import analyze_pose_triage


class CommandRequest(BaseModel):
    action: str
    move_type: Optional[str] = None
    value: Optional[float] = None
    col: Optional[int] = None
    row: Optional[int] = None
    heading_deg: Optional[float] = None


class EndpointUpdateRequest(BaseModel):
    devkit_url: Optional[str] = None
    stream_url: Optional[str] = None


class CalibrationApplyRequest(BaseModel):
    kind: str
    actual_value: float
    commanded_value: float = 90.0
    direction: str = "both"
    lateral_drift_cm: float = 0.0
    speed_pwm: Optional[int] = None
    turn_pwm: Optional[int] = None
    left_trim: Optional[float] = None
    right_trim: Optional[float] = None


class EnvironmentAdaptRequest(BaseModel):
    preset: Optional[str] = None
    width_m: Optional[float] = None
    height_m: Optional[float] = None
    cell_size_m: Optional[float] = None
    lane_stride: Optional[int] = None
    max_coalesced_cells: Optional[int] = None
    fov_assist_visit: Optional[bool] = None
    vo_heading_correction_enabled: Optional[bool] = None
    vo_auto_trim_enabled: Optional[bool] = None
    stop_distance_cm: Optional[float] = None


class SimInjectRequest(BaseModel):
    sim_type: str
    confidence: float = 0.88
    box_height_px: int = 160
    offset_x_px: int = 0


def create_app(
    mission: MissionController,
    dashboard_path: str | pathlib.Path = "dashboard/index.html",
    config_path: str | pathlib.Path = "config.yaml",
    calibration_path: str | pathlib.Path = "calibration.json",
) -> FastAPI:
    app = FastAPI(title="AI Recon Rover v5.6", version="5.6.2")
    dash_file = pathlib.Path(dashboard_path)

    @app.get("/", response_class=HTMLResponse)
    def index() -> HTMLResponse:
        if dash_file.is_file():
            return HTMLResponse(dash_file.read_text(encoding="utf-8"))
        return HTMLResponse("<h1>Dashboard index.html not found</h1>", status_code=404)

    @app.get("/api/state")
    def get_state() -> Dict[str, Any]:
        snap = mission.get_telemetry_snapshot()
        snap["config"] = mission.cfg.to_dict()
        snap["calibration"] = mission.cal.to_dict()
        return snap

    @app.get("/api/targets")
    def get_targets() -> Dict[str, Any]:
        mission.associator.refresh_all_rescue_paths()
        mission.store.save_targets_database(mission.associator.targets)
        return {
            "run_id": mission.store.run_id,
            "home_cell": [mission.cfg.arena.home_col, mission.cfg.arena.home_row],
            "total_targets": len(mission.associator.targets),
            "targets": [t.to_dict() for t in mission.associator.targets],
        }

    @app.get("/api/targets/export.csv")
    def export_targets_csv() -> PlainTextResponse:
        mission.associator.refresh_all_rescue_paths()
        csv_text = mission.store.format_targets_csv(mission.associator.targets)
        return PlainTextResponse(
            content=csv_text,
            media_type="text/csv",
            headers={"Content-Disposition": "attachment; filename=targets_db.csv"},
        )

    @app.post("/api/preflight")
    def api_preflight() -> Dict[str, Any]:
        res = mission.run_preflight()
        return res.to_dict()

    @app.post("/api/config/endpoints")
    def api_update_endpoints(req: EndpointUpdateRequest) -> Dict[str, Any]:
        updated = []
        if req.devkit_url and req.devkit_url.strip():
            new_devkit = req.devkit_url.strip().rstrip("/")
            mission.cfg.robot.devkit_url = new_devkit
            if hasattr(mission.robot, "set_base_url"):
                mission.robot.set_base_url(new_devkit)
            elif hasattr(mission.robot, "base_url"):
                mission.robot.base_url = new_devkit
            updated.append(f"devkit_url={new_devkit}")

        if req.stream_url and req.stream_url.strip():
            new_stream = req.stream_url.strip()
            mission.cfg.camera.stream_url = new_stream
            if hasattr(mission.frame_source, "cfg"):
                mission.frame_source.cfg.stream_url = new_stream
            updated.append(f"stream_url={new_stream}")

        try:
            save_config(mission.cfg, config_path)
        except Exception:
            pass

        msg = "Updated endpoints: " + (", ".join(updated) if updated else "no changes")
        mission.store.log_event("CONFIG_UPDATED", mission.state.value, msg)
        return {
            "ok": True,
            "detail": msg,
            "devkit_url": mission.cfg.robot.devkit_url,
            "stream_url": mission.cfg.camera.stream_url,
        }

    @app.post("/api/calibrate/apply")
    def api_calibrate_apply(req: CalibrationApplyRequest) -> Dict[str, Any]:
        kind = req.kind.strip().lower()
        if kind == "turn":
            apply_turn_calibration(
                mission.cal,
                actual_angle_deg=req.actual_value,
                commanded_angle_deg=req.commanded_value,
                stop_distance_cm=mission.cal.stop_distance_cm,
                direction=req.direction,
            )
        elif kind == "straight":
            dur_s = req.commanded_value / max(1.0, mission.cal.cm_per_s)
            apply_straight_calibration(
                mission.cal,
                measured_distance_cm=req.actual_value,
                test_duration_s=dur_s,
                lateral_drift_cm=req.lateral_drift_cm,
            )
        elif kind == "manual":
            pass
        else:
            raise HTTPException(status_code=400, detail=f"Unknown calibration kind: {kind}")

        if req.speed_pwm is not None:
            mission.cal.speed_pwm = max(95, min(250, int(req.speed_pwm)))
        if req.turn_pwm is not None:
            mission.cal.turn_pwm = max(110, min(250, int(req.turn_pwm)))
        if req.left_trim is not None:
            mission.cal.left_trim = max(0.5, min(1.2, float(req.left_trim)))
        if req.right_trim is not None:
            mission.cal.right_trim = max(0.5, min(1.2, float(req.right_trim)))

        save_calibration(mission.cal, calibration_path)
        synced = False
        sync_note = ""
        try:
            mission.robot.configure(mission.cal)
            synced = True
        except RobotCommunicationError as exc:
            sync_note = f" (saved locally; ESP32 offline: {exc})"

        msg = (
            f"Calibration updated: cm/s={mission.cal.cm_per_s}, deg/s={mission.cal.deg_per_s} "
            f"(L:{mission.cal.effective_deg_per_s(90):.1f}, R:{mission.cal.effective_deg_per_s(-90):.1f}), "
            f"trim L={mission.cal.left_trim} R={mission.cal.right_trim}{sync_note}"
        )
        mission.store.log_event("CALIBRATION_UPDATED", mission.state.value, msg)
        return {"ok": True, "synced": synced, "detail": msg, "calibration": mission.cal.to_dict()}

    @app.post("/api/config/environment")
    def api_config_environment(req: EnvironmentAdaptRequest) -> Dict[str, Any]:
        p = (req.preset or "").strip().lower()
        if p == "rugged_adaptive":
            mission.cfg.arena.width_m = 2.0
            mission.cfg.arena.height_m = 1.5
            mission.cfg.arena.cell_size_m = 0.30
            mission.cfg.planner.lane_stride = 2
            mission.cfg.planner.max_coalesced_cells = 2
            mission.cfg.planner.fov_assist_visit = True
            mission.cfg.vision.vo_heading_correction_enabled = True
            mission.cfg.vision.vo_auto_trim_enabled = True
            mission.cal.stop_distance_cm = 24.0
        elif p == "compact_demo":
            mission.cfg.arena.width_m = 1.2
            mission.cfg.arena.height_m = 0.9
            mission.cfg.arena.cell_size_m = 0.30
            mission.cfg.planner.lane_stride = 2
            mission.cfg.planner.max_coalesced_cells = 2
            mission.cfg.planner.fov_assist_visit = True
            mission.cfg.vision.vo_heading_correction_enabled = True
            mission.cfg.vision.vo_auto_trim_enabled = True
            mission.cal.stop_distance_cm = 24.0
        elif p == "dense_grid":
            mission.cfg.arena.width_m = 2.0
            mission.cfg.arena.height_m = 1.5
            mission.cfg.arena.cell_size_m = 0.30
            mission.cfg.planner.lane_stride = 1
            mission.cfg.planner.max_coalesced_cells = 2
            mission.cfg.planner.fov_assist_visit = False
            mission.cfg.vision.vo_heading_correction_enabled = True
            mission.cfg.vision.vo_auto_trim_enabled = True

        if req.width_m is not None and req.width_m >= 0.6:
            mission.cfg.arena.width_m = round(float(req.width_m), 2)
        if req.height_m is not None and req.height_m >= 0.6:
            mission.cfg.arena.height_m = round(float(req.height_m), 2)
        if req.cell_size_m is not None and req.cell_size_m >= 0.20:
            mission.cfg.arena.cell_size_m = round(float(req.cell_size_m), 2)
        if req.lane_stride is not None:
            mission.cfg.planner.lane_stride = max(1, min(3, int(req.lane_stride)))
        if req.max_coalesced_cells is not None:
            mission.cfg.planner.max_coalesced_cells = max(1, min(4, int(req.max_coalesced_cells)))
        if req.fov_assist_visit is not None:
            mission.cfg.planner.fov_assist_visit = bool(req.fov_assist_visit)
        if req.vo_heading_correction_enabled is not None:
            mission.cfg.vision.vo_heading_correction_enabled = bool(req.vo_heading_correction_enabled)
        if req.vo_auto_trim_enabled is not None:
            mission.cfg.vision.vo_auto_trim_enabled = bool(req.vo_auto_trim_enabled)
        if req.stop_distance_cm is not None and req.stop_distance_cm >= 12.0:
            mission.cal.stop_distance_cm = round(float(req.stop_distance_cm), 1)
            mission.cfg.robot.stop_distance_cm = mission.cal.stop_distance_cm

        mission.rebuild_grid()
        try:
            save_config(mission.cfg, config_path)
            save_calibration(mission.cal, calibration_path)
        except Exception:
            pass
        try:
            mission.robot.configure(mission.cal)
        except Exception:
            pass

        msg = (
            f"Environment updated: Arena {mission.cfg.arena.width_m}x{mission.cfg.arena.height_m}m, "
            f"lane_stride={mission.cfg.planner.lane_stride}, coalesced={mission.cfg.planner.max_coalesced_cells}, "
            f"fov_assist={mission.cfg.planner.fov_assist_visit}, VO_drift_fix={mission.cfg.vision.vo_heading_correction_enabled}, "
            f"stop_dist={mission.cal.stop_distance_cm}cm"
        )
        mission.store.log_event("ENV_ADAPTED", mission.state.value, msg)
        return {"ok": True, "detail": msg, "config": mission.cfg.to_dict(), "calibration": mission.cal.to_dict()}

    @app.post("/api/command")
    def api_command(req: CommandRequest) -> Dict[str, Any]:
        act = req.action.strip().lower()
        try:
            if act == "start":
                pf = mission.start_mission(background=True)
                return {"ok": pf.passed, "state": mission.state.value, "preflight": pf.to_dict()}
            if act == "pause":
                mission.pause()
                return {"ok": True, "state": mission.state.value}
            if act == "resume":
                mission.resume(background=True)
                return {"ok": True, "state": mission.state.value}
            if act == "abort":
                mission.abort()
                return {"ok": True, "state": mission.state.value}
            if act == "return_home":
                mission.trigger_return_home(background=True)
                return {"ok": True, "state": mission.state.value}
            if act == "e_stop":
                st = mission.e_stop()
                return {"ok": True, "state": mission.state.value, "robot_status": st.to_dict()}
            if act == "reset_estop":
                st = mission.reset_estop()
                return {"ok": True, "state": mission.state.value, "robot_status": st.to_dict()}
            if act == "scan_now":
                cands = mission.manual_scan_now()
                return {
                    "ok": True,
                    "state": mission.state.value,
                    "candidates": [c.to_dict() for c in cands],
                    "targets": [t.to_dict() for t in mission.associator.targets],
                    "latest_alert": mission.latest_alert,
                }
            if act == "ibvs_align":
                return mission.execute_ibvs_alignment()
            if act == "goto_cell":
                if req.col is None or req.row is None:
                    raise HTTPException(status_code=400, detail="col and row required for goto_cell")
                return mission.goto_cell(int(req.col), int(req.row), background=True)
            if act == "set_gear_pwm":
                pwm_val = max(95, min(250, int(req.value if req.value is not None else 170)))
                mission.cal.speed_pwm = pwm_val
                try:
                    save_calibration(mission.cal, calibration_path)
                except Exception:
                    pass
                try:
                    mission.robot.configure(mission.cal)
                    return {"ok": True, "state": mission.state.value, "speed_pwm": pwm_val, "synced": True}
                except RobotCommunicationError as exc:
                    return {
                        "ok": True,
                        "state": mission.state.value,
                        "speed_pwm": pwm_val,
                        "synced": False,
                        "warning": f"Saved speed_pwm={pwm_val} locally (ESP32 offline: {exc})",
                    }
            if act == "manual_move":
                mtype = req.move_type or "forward"
                val = float(req.value if req.value is not None else 15.0)
                st = mission.manual_move(mtype, val)
                return {"ok": True, "state": mission.state.value, "robot_status": st.to_dict()}
        except RobotCommunicationError as exc:
            err_msg = (
                f"ESP32 communication timeout ({mission.cfg.robot.devkit_url}). "
                f"Check that ESP32 is powered on, connected to hotspot, and IP matches config.yaml. Details: {exc}"
            )
            mission.store.log_event("ESP32_COMMS_WARN", mission.state.value, err_msg)
            return {
                "ok": False,
                "state": mission.state.value,
                "error": err_msg,
                "detail": str(exc),
            }
        except HTTPException:
            raise
        except Exception as exc:
            err_msg = f"Command '{act}' failed: {exc}"
            mission.store.log_event("COMMAND_ERROR", mission.state.value, err_msg)
            return {
                "ok": False,
                "state": mission.state.value,
                "error": err_msg,
                "detail": str(exc),
            }
        raise HTTPException(status_code=400, detail=f"Unknown command: {act}")

    @app.post("/api/sim/inject")
    def api_sim_inject(req: SimInjectRequest) -> Dict[str, Any]:
        st = req.sim_type.strip().lower()
        if st == "person_confirmed":
            cands = mission.simulate_person_sighting(
                confidence=req.confidence, box_height_px=req.box_height_px, offset_x_px=req.offset_x_px
            )
            return {
                "ok": True,
                "candidates": [c.to_dict() for c in cands],
                "targets": [t.to_dict() for t in mission.associator.targets],
                "latest_alert": mission.latest_alert,
            }
        if st == "person_fallen":
            cands = mission.simulate_person_sighting(
                confidence=0.93, box_height_px=95, box_width_px=145, offset_x_px=req.offset_x_px
            )
            return {
                "ok": True,
                "candidates": [c.to_dict() for c in cands],
                "targets": [t.to_dict() for t in mission.associator.targets],
                "latest_alert": mission.latest_alert,
            }
        if st == "person_tentative":
            cands = mission.simulate_person_sighting(
                confidence=0.27, box_height_px=30, offset_x_px=req.offset_x_px
            )
            return {
                "ok": True,
                "candidates": [c.to_dict() for c in cands],
                "pending_ai_confirm_cell": list(mission.pending_ai_confirm_cell)
                if mission.pending_ai_confirm_cell
                else None,
            }
        return {"ok": False, "detail": "unknown_sim_type"}

    @app.get("/report", response_class=HTMLResponse)
    def printable_report() -> HTMLResponse:
        snap = mission.get_telemetry_snapshot()
        cov = snap.get("coverage", {})
        targets = snap.get("targets", [])
        rows_html = (
            "".join(
                [
                    f"<tr><td><strong>{t['target_id']}</strong></td><td>{t['triage_priority']} ({t['posture_label']})</td><td>({t['x_m']:.2f}m, {t['y_m']:.2f}m) Cell [{t['col']},{t['row']}]</td><td>{t['distance_from_home_m']:.2f}m (A*: {t['rescue_path_distance_m']:.2f}m)</td><td><code>{t['rescue_directions']}</code></td><td>{t['confidence']*100:.0f}%</td></tr>"
                    for t in targets
                ]
            )
            or "<tr><td colspan='6'>No targets logged yet.</td></tr>"
        )

        html = f"""<!DOCTYPE html><html><head><meta charset='utf-8'><title>Mission Evaluation Report — {snap['run_id']}</title>
<style>body{{font-family:system-ui,-apple-system,sans-serif;color:#0f172a;max-width:960px;margin:28px auto;padding:0 20px;line-height:1.5;}}h1{{color:#0284c7;border-bottom:2px solid #0284c7;padding-bottom:8px;}}table{{width:100%;border-collapse:collapse;margin-top:12px;font-size:13px;}}th,td{{border:1px solid #cbd5e1;padding:8px 10px;text-align:left;}}th{{background:#f1f5f9;}}.grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:16px 0;}}.box{{background:#f8fafc;border:1px solid #cbd5e1;border-radius:8px;padding:12px;}}.box .lbl{{font-size:11px;text-transform:uppercase;color:#64748b;font-weight:700;}}.box .val{{font-size:18px;font-weight:800;color:#0f172a;margin-top:4px;}}.print-btn{{background:#0284c7;color:#fff;border:none;padding:8px 16px;border-radius:6px;font-weight:700;cursor:pointer;float:right;}}@media print{{.print-btn{{display:none;}}}}</style></head><body>
<button class='print-btn' onclick='window.print()'>Print / Save PDF</button>
<h1>AI-Assisted Search & Reconnaissance Rover — Mission Evaluation Report</h1>
<p><strong>Run ID:</strong> {snap['run_id']} | <strong>State:</strong> {snap['state']} | <strong>Inference Engine:</strong> YOLOv8-Pose ({snap['detector_device']})</p>
<div class='grid'>
<div class='box'><div class='lbl'>Arena Area</div><div class='val'>{mission.cfg.arena.width_m:.2f}m &times; {mission.cfg.arena.height_m:.2f}m</div></div>
<div class='box'><div class='lbl'>Visited Coverage</div><div class='val'>{cov.get('visited_coverage_pct', 0.0):.1f}% ({cov.get('visited_cells', 0)}/{cov.get('eligible_cells', 0)})</div></div>
<div class='box'><div class='lbl'>Observed FOV</div><div class='val'>{cov.get('observed_coverage_pct', 0.0):.1f}% ({cov.get('observed_cells', 0)}/{cov.get('eligible_cells', 0)})</div></div>
<div class='box'><div class='lbl'>Survivors Logged</div><div class='val'>{len(targets)} Targets</div></div>
</div>
<h2>1. Dual-Layer Boundary System</h2>
<p>1. <strong>Software Virtual Geofence:</strong> Discretizes the {mission.cfg.arena.width_m:.1f}m &times; {mission.cfg.arena.height_m:.1f}m search zone into {cov.get('cols', 0)}&times;{cov.get('rows', 0)} grid cells ({mission.cfg.arena.cell_size_m:.2f}m each) with safety margins, restricting all planned Boustrophedon and A* motion inside bounds.<br/>2. <strong>Hardware Ultrasonic Guard:</strong> Non-blocking sampling of front HC-SR04 every 85 ms halts motion immediately if an obstacle is within {mission.cal.stop_distance_cm:.0f} cm.</p>
<h2>2. Confirmed Survivor Triage & A* Rescue Dispatch</h2>
<table><thead><tr><th>Target ID</th><th>Posture Triage</th><th>Coordinates & Cell</th><th>Distance from Home</th><th>A* Rescue Route</th><th>Confidence</th></tr></thead><tbody>{rows_html}</tbody></table>
</body></html>"""
        return HTMLResponse(content=html)

    def _mjpeg_generator():
        frame_counter = 0
        live_preview_boxes = []
        last_live_detect_ts = 0.0
        while True:
            frame, ts, _ = mission.frame_source.get_latest_frame()
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    "WAITING FOR CAMERA STREAM...",
                    (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 140, 255),
                    2,
                )
            else:
                frame = frame.copy()
                h_img, w_img = frame.shape[:2]
                frame_counter += 1
                now_t = time.time()
                if now_t - last_live_detect_ts >= 0.35 and mission.state.value in ("IDLE", "PAUSED", "PREFLIGHT"):
                    try:
                        det_boxes = mission.detector.detect(frame, frame_index=frame_counter)
                        live_preview_boxes = [b.to_dict() for b in det_boxes]
                        last_live_detect_ts = now_t
                    except Exception:
                        pass

                active_boxes = (
                    mission.latest_annotated_boxes
                    if mission.state.value not in ("IDLE", "PAUSED") and mission.latest_annotated_boxes
                    else (live_preview_boxes or mission.latest_annotated_boxes)
                )

                cv2.rectangle(frame, (0, 0), (w_img, 28), (15, 23, 42), -1)
                us_cm = getattr(mission.robot, "last_status", None)
                us_val = us_cm.ultrasonic_cm if us_cm else -1.0
                cx_cell, cy_cell = mission.current_cell()
                hud_txt = f"HUD | STATE: {mission.state.value} | CELL: [{cx_cell},{cy_cell}] | US: {us_val:.1f}cm | YOLO: {mission.detector.device_name}"
                cv2.putText(frame, hud_txt, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (56, 189, 248), 1, cv2.LINE_AA)

                cx_mid, cy_mid = w_img // 2, h_img // 2
                cv2.line(frame, (cx_mid - 14, cy_mid), (cx_mid + 14, cy_mid), (100, 116, 139), 1)
                cv2.line(frame, (cx_mid, cy_mid - 14), (cx_mid, cy_mid + 14), (100, 116, 139), 1)

                # Draw Semantic Ground Hazard Zones (if red/orange marker detected in lower frame)
                if mission.cfg.vision.hazard_detection_enabled:
                    for hz in detect_ground_hazards(frame):
                        cv2.rectangle(frame, (hz["x1"], hz["y1"]), (hz["x2"], hz["y2"]), (0, 110, 255), 2)
                        cv2.rectangle(
                            frame,
                            (hz["x1"], max(28, hz["y1"] - 20)),
                            (min(w_img - 2, hz["x1"] + 210), max(28, hz["y1"])),
                            (0, 90, 230),
                            -1,
                        )
                        cv2.putText(
                            frame,
                            "HAZARD ZONE (AVOID)",
                            (hz["x1"] + 4, max(42, hz["y1"] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.42,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )

                SKELETON_EDGES = [
                    (5, 6),
                    (5, 7),
                    (7, 9),
                    (6, 8),
                    (8, 10),
                    (5, 11),
                    (6, 12),
                    (11, 12),
                    (11, 13),
                    (13, 15),
                    (12, 14),
                    (14, 16),
                ]
                for b in active_boxes:
                    x1, y1, x2, y2 = int(b["x1"]), int(b["y1"]), int(b["x2"]), int(b["y2"])
                    kpts = b.get("keypoints", [])
                    dbox = DetectionBox(x1=x1, y1=y1, x2=x2, y2=y2, confidence=b["confidence"], keypoints=kpts)
                    posture_lbl, prio_lbl, is_wav, t_ang, _ = analyze_pose_triage(dbox)

                    bh = max(1.0, float(y2 - y1))
                    bw_center = 0.5 * (x1 + x2)
                    est_dist_m = max(
                        0.25,
                        min(
                            3.5,
                            (mission.cfg.vision.focal_scale_px * mission.cfg.vision.assumed_person_height_m)
                            / (bh * 3.0),
                        ),
                    )
                    est_brg_deg = ((bw_center - (w_img / 2.0)) / max(1.0, float(w_img))) * mission.cfg.camera.hfov_deg

                    box_color = (
                        (36, 36, 240)
                        if prio_lbl == "P1-CRITICAL"
                        else ((0, 180, 255) if prio_lbl == "P2-URGENT" else (0, 230, 64))
                    )
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

                    # Draw 17-keypoint neon skeleton overlay if pose keypoints are present
                    if kpts and len(kpts) >= 17:
                        for i, j in SKELETON_EDGES:
                            if kpts[i][2] > 0.30 and kpts[j][2] > 0.30:
                                pt1 = (int(kpts[i][0]), int(kpts[i][1]))
                                pt2 = (int(kpts[j][0]), int(kpts[j][1]))
                                cv2.line(frame, pt1, pt2, (255, 220, 0), 2, cv2.LINE_AA)
                        for kx, ky, kc in kpts:
                            if kc > 0.30:
                                cv2.circle(frame, (int(kx), int(ky)), 3, (0, 255, 255), -1)

                    label_top = f"{prio_lbl} ({posture_lbl}) {b['confidence']*100:.0f}% | {est_dist_m:.2f}m | IBVS:{-0.85*est_brg_deg:+.1f}deg"
                    cv2.rectangle(
                        frame,
                        (x1, max(28, y1 - 22)),
                        (min(w_img - 2, x1 + 410), max(28, y1)),
                        (0, 100, 35) if prio_lbl == "P3-AMBULATORY" else (20, 20, 180),
                        -1,
                    )
                    cv2.putText(
                        frame,
                        label_top,
                        (x1 + 4, max(44, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                payload = jpg.tobytes()
                header = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(payload)).encode("ascii")
                    + b"\r\n\r\n"
                )
                yield header + payload + b"\r\n"
            time.sleep(0.10)

    @app.get("/video")
    def video_stream() -> StreamingResponse:
        return StreamingResponse(_mjpeg_generator(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.websocket("/ws")
    async def websocket_telemetry(ws: WebSocket) -> None:
        await ws.accept()
        interval = 1.0 / max(1.0, mission.cfg.server.ws_telemetry_hz)
        try:
            while True:
                await ws.send_json(mission.get_telemetry_snapshot())
                await asyncio.sleep(interval)
        except Exception:
            return

    return app
