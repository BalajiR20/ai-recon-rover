"""Mission state machine, Sense-Think-Act loop, Smart AI Confirmation, IBVS, and Return-Home."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional, Set

from rover.association import TargetAssociator
from rover.config import AppConfig, CalibrationData
from rover.detector import PersonDetector, compute_optical_flow_shift, detect_ground_hazards, estimate_visual_heading_drift_deg
from rover.frames import FrameSource
from rover.grid import ArenaGrid, GridCoord
from rover.localization import PoseEstimator, angle_diff_deg
from rover.models import (
    CellStatus,
    DetectionBox,
    EvidenceMode,
    MissionReport,
    MissionState,
    PreflightResult,
    RobotStatus,
    SightingCandidate,
)
from rover.preflight import run_preflight_checks
from rover.robot import Esp32Robot, RobotCommunicationError, RobotInterface
from rover.store import MissionStore
from rover.tracker import MultiFrameBurstTracker


class InvalidTransitionError(RuntimeError):
    pass


class MissionController:
    def __init__(
        self,
        cfg: AppConfig,
        cal: CalibrationData,
        robot: RobotInterface,
        frame_source: FrameSource,
        detector: PersonDetector,
        store: MissionStore,
        evidence_mode: EvidenceMode = EvidenceMode.MOCK,
        fast_scan_timing: bool = False,
    ) -> None:
        self.cfg = cfg
        self.cal = cal
        self.robot = robot
        self.frame_source = frame_source
        self.detector = detector
        self.store = store
        self.evidence_mode = evidence_mode
        self.fast_scan_timing = fast_scan_timing

        self.grid = ArenaGrid(cfg.arena, cfg.planner, cfg.camera)
        self.pose_est = PoseEstimator(
            self.grid,
            cfg.localization,
            cfg.arena,
            is_simulated=(evidence_mode != EvidenceMode.REAL_HARDWARE),
        )
        self.tracker = MultiFrameBurstTracker(self.grid, cfg.vision, cfg.camera)
        self.associator = TargetAssociator(self.grid, cfg.association)

        self.state: MissionState = MissionState.IDLE
        self._lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        self.started_at: float = 0.0
        self.ended_at: float = 0.0
        self.step_count: int = 0
        self.steps_without_progress: int = 0
        self.cell_revisit_streak: Dict[GridCoord, int] = {}
        self.consecutive_invalid_us: int = 0
        self.fault_count: int = 0
        self.recovery_count: int = 0
        self.tentative_sightings_count: int = 0
        self.smart_confirmations_triggered: int = 0

        self.planned_path: List[GridCoord] = []
        self.current_target_cell: Optional[GridCoord] = None
        self.pending_ai_confirm_cell: Optional[GridCoord] = None
        self.ai_confirm_attempts_at_cell: Set[GridCoord] = set()
        self.last_command_summary: Dict[str, Any] = {}
        self.last_preflight: Optional[PreflightResult] = None
        self.latest_annotated_boxes: List[Any] = []
        self.latest_alert: Optional[Dict[str, Any]] = None
        self.alert_seq: int = 0
        self.returning_home_mode: bool = False
        self.goto_goal: Optional[GridCoord] = None
        self.last_vo_status: Dict[str, Any] = {
            "status": "STANDBY",
            "magnitude_px": 0.0,
            "dx_px": 0.0,
            "drift_deg": 0.0,
            "correction_deg": 0.0,
        }

    def rebuild_grid(self) -> None:
        """Hot-rebuild ArenaGrid, PoseEstimator, Tracker, and Associator after arena/planner preset change."""
        self.grid = ArenaGrid(self.cfg.arena, self.cfg.planner, self.cfg.camera)
        self.pose_est = PoseEstimator(
            self.grid,
            self.cfg.localization,
            self.cfg.arena,
            is_simulated=(self.evidence_mode != EvidenceMode.REAL_HARDWARE),
        )
        self.tracker = MultiFrameBurstTracker(self.grid, self.cfg.vision, self.cfg.camera)
        self.associator.grid = self.grid
        self.associator.refresh_all_rescue_paths()
        self.planned_path = []
        self.current_target_cell = None

    def transition_to(self, new_state: MissionState, reason: str = "") -> None:
        with self._lock:
            if new_state == self.state:
                return
            old_state = self.state
            self.state = new_state
            if new_state == MissionState.FAULT:
                self.fault_count += 1
            elif new_state == MissionState.RECOVERY_REQUIRED:
                self.recovery_count += 1
            self.store.log_event(
                "STATE_CHANGE",
                new_state.value,
                f"{old_state.value} -> {new_state.value}" + (f" ({reason})" if reason else ""),
                {"from": old_state.value, "to": new_state.value, "reason": reason},
            )

    def current_cell(self) -> GridCoord:
        return self.grid.world_to_cell(self.pose_est.pose.x_m, self.pose_est.pose.y_m)

    def run_preflight(self) -> PreflightResult:
        if self.state in (MissionState.MISSION_COMPLETE, MissionState.MISSION_ABORTED):
            self.transition_to(MissionState.IDLE, "reset_for_preflight")
        if self.state == MissionState.IDLE:
            self.transition_to(MissionState.PREFLIGHT, "running_preflight")
        res = run_preflight_checks(
            self.cfg, self.cal, self.grid, self.robot, self.frame_source, self.detector
        )
        self.last_preflight = res
        self.store.log_event(
            "PREFLIGHT_RESULT",
            self.state.value,
            f"Preflight {'PASSED' if res.passed else 'FAILED'}",
            res.to_dict(),
        )
        if self.state == MissionState.PREFLIGHT:
            self.transition_to(MissionState.IDLE, "preflight_passed_ready" if res.passed else "preflight_failed")
        return res

    def start_mission(self, background: bool = True) -> PreflightResult:
        res = self.run_preflight()
        if not res.passed:
            return res

        self.started_at = time.time()
        self.ended_at = 0.0
        self.step_count = 0
        self.steps_without_progress = 0
        self.cell_revisit_streak.clear()
        self.consecutive_invalid_us = 0
        self.returning_home_mode = False
        self.pending_ai_confirm_cell = None
        self._stop_flag.clear()

        home_cell = (self.cfg.arena.home_col, self.cfg.arena.home_row)
        self.grid.mark_visited(*home_cell)
        self.transition_to(MissionState.PLANNING, "preflight_passed")

        if background:
            self._worker_thread = threading.Thread(target=self._mission_loop, name="MissionLoopWorker", daemon=True)
            self._worker_thread.start()
        else:
            self._mission_loop()
        return res

    def _mission_loop(self) -> None:
        try:
            if self.state == MissionState.PLANNING:
                self._perform_scan_at_current_cell(reason="initial_home_scan")

            while not self._stop_flag.is_set():
                if self.state in (
                    MissionState.PAUSED,
                    MissionState.RECOVERY_REQUIRED,
                    MissionState.IDLE,
                    MissionState.MISSION_COMPLETE,
                    MissionState.MISSION_ABORTED,
                    MissionState.FAULT,
                ):
                    break
                if not self.step_once():
                    break
        except Exception as exc:
            self.store.log_event("UNHANDLED_EXCEPTION", self.state.value, str(exc))
            try:
                self.robot.stop()
            except Exception:
                pass
            self.state = MissionState.FAULT
        finally:
            self.ended_at = time.time()
            self.save_artifacts()

    def step_once(self) -> bool:
        elapsed_s = time.time() - (self.started_at or time.time())
        stats = self.grid.compute_coverage_stats()
        max_steps = max(10, self.cfg.mission.max_steps_multiplier * max(1, stats.eligible_cells))

        if not self.returning_home_mode:
            if elapsed_s >= self.cfg.mission.max_mission_time_s:
                self._initiate_return_home("time_budget_reached")
            elif self.step_count >= max_steps:
                self._initiate_return_home("step_budget_reached")
            elif self.steps_without_progress >= self.cfg.mission.max_steps_without_progress:
                self._initiate_return_home("stuck_watchdog")

        try:
            st = self.robot.get_status()
        except RobotCommunicationError as exc:
            self.store.log_event("COMMS_LOST", self.state.value, str(exc))
            self.transition_to(MissionState.RECOVERY_REQUIRED, "comms_lost")
            return False

        if isinstance(self.robot, Esp32Robot) and self.robot.reboot_detected:
            self.robot.configure(self.cal)
            self.transition_to(MissionState.RECOVERY_REQUIRED, "board_reboot")
            return False

        if st.state == "estopped":
            self.transition_to(MissionState.MISSION_ABORTED, "hardware_estopped")
            return False

        curr = self.current_cell()
        if self.returning_home_mode:
            if self.state != MissionState.RETURNING_HOME:
                self.transition_to(MissionState.RETURNING_HOME, "returning_home")
            home = (self.cfg.arena.home_col, self.cfg.arena.home_row)
            if curr == home or self.pose_est.is_home_reached_estimated():
                self.transition_to(MissionState.MISSION_COMPLETE, "home_reached")
                return False
            path = self.grid.astar_path(curr, home)
            if path is None or len(path) < 2:
                self.transition_to(MissionState.RECOVERY_REQUIRED, "return_home_blocked")
                return False
            self.planned_path = path
            return self._execute_step_to_neighbor(curr, path[1], is_return_home=True)

        if self.state != MissionState.PLANNING:
            self.transition_to(MissionState.PLANNING, "planning_next_cell")

        target_cell, path, is_ai_confirm = self.grid.next_coverage_target(
            curr, pending_ai_confirm_cell=self.pending_ai_confirm_cell
        )
        if target_cell is None or path is None:
            self._initiate_return_home("coverage_complete")
            return True

        self.current_target_cell = target_cell
        self.planned_path = path

        if len(path) == 1 or target_cell == curr:
            self.grid.mark_visited(*curr)
            if is_ai_confirm:
                self.pending_ai_confirm_cell = None
                self.smart_confirmations_triggered += 1
            self._perform_scan_at_current_cell(reason="ai_confirmation_inplace" if is_ai_confirm else "current_cell_scan")
            return True

        # Coalesce collinear straight cells up to max_coalesced_cells to reduce start/stop caster wiggle
        max_coal = max(1, int(self.cfg.planner.max_coalesced_cells))
        coal_cells: List[GridCoord] = [path[1]]
        if max_coal > 1 and len(path) >= 3 and not is_ai_confirm:
            dc0 = path[1][0] - curr[0]
            dr0 = path[1][1] - curr[1]
            for idx in range(2, min(len(path), max_coal + 1)):
                prev_c = path[idx - 1]
                nxt_c = path[idx]
                if (nxt_c[0] - prev_c[0], nxt_c[1] - prev_c[1]) == (dc0, dr0):
                    if self.grid.should_scan_at(*prev_c):
                        break
                    coal_cells.append(nxt_c)
                else:
                    break
        return self._execute_step_to_neighbor(curr, coal_cells[-1], is_return_home=False, is_ai_confirm=is_ai_confirm, intermediate_cells=coal_cells[:-1])

    def apply_visual_drift_correction(
        self,
        pre_frame: Any,
        post_frame: Any,
    ) -> Dict[str, Any]:
        """
        Closed-loop Visual Odometry Drift Sensing & Correction:
        Measures unintended horizontal yaw drift during a straight move using Farneback flow.
        If drift exceeds threshold, executes a corrective micro-turn and adapts motor trim online.
        """
        fdx, fdy, fmag, drift_deg = estimate_visual_heading_drift_deg(
            pre_frame, post_frame, hfov_deg=self.cfg.camera.hfov_deg
        )
        vo_state = (
            "MOTION_CONFIRMED"
            if (fmag >= 0.8 or self.evidence_mode != EvidenceMode.REAL_HARDWARE)
            else "LOW_FLOW_SLIP_CHECK"
        )
        corr_deg = 0.0
        if (
            self.cfg.vision.vo_heading_correction_enabled
            and abs(drift_deg) >= 2.2
            and fmag >= 0.8
        ):
            corr_deg = round(-drift_deg, 1)
            vo_state = f"DRIFT_FIXED ({corr_deg:+.1f} deg)"
            try:
                self.robot.execute_move("turn", corr_deg)
            except Exception:
                pass
            if self.cfg.vision.vo_auto_trim_enabled and abs(drift_deg) >= 2.5:
                if drift_deg > 0:
                    # Rover curved Left -> right wheel is slightly faster than left wheel
                    self.cal.right_trim = round(max(0.70, self.cal.right_trim - 0.008), 3)
                else:
                    # Rover curved Right -> left wheel is slightly faster than right wheel
                    self.cal.left_trim = round(max(0.70, self.cal.left_trim - 0.008), 3)
                try:
                    self.robot.configure(self.cal)
                except Exception:
                    pass
            self.store.log_event(
                "VO_DRIFT_CORRECTED",
                self.state.value,
                f"Visual Odometry sensed {drift_deg:+.1f}° drift (dx={fdx:+.1f}px) -> applied {corr_deg:+.1f}° corrective micro-turn (trim L={self.cal.left_trim}, R={self.cal.right_trim})",
                {"dx_px": fdx, "drift_deg": drift_deg, "correction_deg": corr_deg, "left_trim": self.cal.left_trim, "right_trim": self.cal.right_trim},
            )

        self.last_vo_status = {
            "status": vo_state,
            "magnitude_px": fmag,
            "dx_px": fdx,
            "drift_deg": drift_deg,
            "correction_deg": corr_deg,
        }
        return self.last_vo_status

    def _execute_step_to_neighbor(
        self,
        curr: GridCoord,
        dest: GridCoord,
        is_return_home: bool = False,
        is_ai_confirm: bool = False,
        intermediate_cells: Optional[List[GridCoord]] = None,
    ) -> bool:
        wx, wy = self.grid.cell_to_world(*dest)
        dx = wx - self.pose_est.pose.x_m
        dy = wy - self.pose_est.pose.y_m
        desired_heading = math.degrees(math.atan2(dy, dx))
        heading_error = angle_diff_deg(desired_heading, self.pose_est.pose.heading_deg)

        if abs(heading_error) >= 10.0:
            st_turn = self.robot.execute_move("turn", heading_error)
            self.pose_est.update_from_turn(st_turn.executed_deg)
            if not self.fast_scan_timing and self.evidence_mode == EvidenceMode.REAL_HARDWARE:
                time.sleep(0.15)
            if abs(angle_diff_deg(desired_heading, self.pose_est.pose.heading_deg)) >= 25.0:
                return True

        # Proactive post-turn ultrasonic check: if obstacle is already within stop threshold, replan before lunging
        if self.evidence_mode == EvidenceMode.REAL_HARDWARE:
            try:
                st_pre = self.robot.get_status()
                if 2.0 < st_pre.ultrasonic_cm <= (self.cal.stop_distance_cm + 3.0):
                    block_cell = intermediate_cells[0] if intermediate_cells else dest
                    self.grid.mark_discovered_obstacle(block_cell[0], block_cell[1], current_cell=self.current_cell())
                    self.associator.refresh_all_rescue_paths()
                    self.store.save_targets_database(self.associator.targets)
                    self.store.log_event(
                        "PROACTIVE_OBSTACLE_AVOID",
                        self.state.value,
                        f"Ultrasonic sensed obstacle at {st_pre.ultrasonic_cm:.1f}cm ahead before forward step; marked {block_cell} BLOCKED and replanning",
                    )
                    if not is_return_home:
                        self.transition_to(MissionState.REPLANNING, "proactive_obstacle")
                    return True
            except Exception:
                pass

        pre_frame, _, _ = self.frame_source.get_latest_frame()
        dist_cm = math.hypot(dx, dy) * 100.0
        st_fwd = self.robot.execute_move("forward", dist_cm)
        self.pose_est.update_from_translation(st_fwd.executed_cm, direction="forward")
        if self.cfg.vision.optical_flow_enabled and pre_frame is not None:
            post_frame, _, _ = self.frame_source.get_latest_frame()
            self.apply_visual_drift_correction(pre_frame, post_frame)
        self.step_count += 1

        if st_fwd.state == "aborted_obstacle":
            block_cell = intermediate_cells[0] if (intermediate_cells and st_fwd.executed_cm < self.cfg.arena.cell_size_m * 80.0) else dest
            self.grid.mark_discovered_obstacle(block_cell[0], block_cell[1], current_cell=self.current_cell())
            self.associator.refresh_all_rescue_paths()
            self.store.save_targets_database(self.associator.targets)
            if not is_return_home:
                self.transition_to(MissionState.REPLANNING, "aborted_obstacle")
            return True

        if intermediate_cells:
            for ic in intermediate_cells:
                self.grid.mark_visited(*ic)
        new_cell = self.current_cell()
        self.grid.mark_visited(*new_cell)

        if is_ai_confirm:
            self.pending_ai_confirm_cell = None
            self.smart_confirmations_triggered += 1
            self._perform_scan_at_current_cell(reason="ai_confirmation_step")
        elif not is_return_home and self.grid.should_scan_at(*new_cell):
            self._perform_scan_at_current_cell(reason="scheduled_waypoint")

        return True

    def _perform_scan_at_current_cell(self, reason: str = "scheduled") -> List[SightingCandidate]:
        if self.state != MissionState.SCANNING:
            self.transition_to(MissionState.SCANNING, reason)

        dwell_s = 0.01 if self.fast_scan_timing else (self.cfg.vision.settle_s + self.cfg.vision.camera_latency_s)
        time.sleep(dwell_s)

        burst_detections = []
        last_frame = None
        for f_idx in range(self.cfg.vision.scan_frames):
            frame, _, _ = self.frame_source.get_latest_frame()
            if frame is not None:
                last_frame = frame
                boxes = self.detector.detect(frame, frame_index=f_idx)
                burst_detections.append(boxes)
            if not self.fast_scan_timing and f_idx + 1 < self.cfg.vision.scan_frames:
                time.sleep(0.05)

        self.grid.raycast_observed_fov(self.pose_est.pose.x_m, self.pose_est.pose.y_m, self.pose_est.pose.heading_deg)
        if last_frame is not None and self.cfg.vision.hazard_detection_enabled:
            hazards = detect_ground_hazards(last_frame)
            if hazards:
                curr = self.current_cell()
                ahead = self.grid.cell_ahead(curr[0], curr[1], self.pose_est.pose.heading_deg)
                if self.grid.mark_discovered_hazard(ahead[0], ahead[1], current_cell=curr):
                    self.associator.refresh_all_rescue_paths()
                    self.store.save_targets_database(self.associator.targets)
                    self.store.log_event(
                        "HAZARD_DETECTED",
                        self.state.value,
                        f"Semantic ground hazard detected visually; marked cell {ahead} as HAZARD and updated A* routes",
                        {"cell": list(ahead), "hazards": hazards},
                    )
        candidates = self.tracker.evaluate_burst(burst_detections, self.pose_est.pose)
        confirmed = [c for c in candidates if c.confirmed]
        tentative = [c for c in candidates if c.tentative]

        if last_frame is not None and (confirmed or tentative):
            for cand in confirmed + tentative:
                tag = "confirmed" if cand.confirmed else "tentative"
                cand.snapshot_path = self.store.save_snapshot(
                    last_frame,
                    cand.boxes,
                    label_prefix=tag,
                    caption=f"{tag.upper()} conf={cand.mean_confidence:.2f} dist={cand.est_distance_m:.2f}m",
                )

        if confirmed:
            records = self.associator.associate_burst_candidates(confirmed, self.pose_est.pose)
            self.store.save_targets_database(self.associator.targets)
            self.latest_annotated_boxes = [b.to_dict() for c in confirmed for b in c.boxes]
            for rec in records:
                self.alert_seq += 1
                self.latest_alert = {
                    "alert_seq": self.alert_seq,
                    "timestamp": time.time(),
                    "target_id": rec.target_id,
                    "association_label": rec.association_label,
                    "triage_priority": rec.triage_priority,
                    "posture_label": rec.posture_label,
                    "x_m": rec.x_m,
                    "y_m": rec.y_m,
                    "col": rec.col,
                    "row": rec.row,
                    "confidence": rec.confidence,
                    "distance_from_home_m": rec.distance_from_home_m,
                    "rescue_path_distance_m": rec.rescue_path_distance_m,
                    "rescue_directions": rec.rescue_directions,
                    "path_from_home": rec.path_from_home,
                    "ibvs_turn_deg": rec.ibvs_turn_deg,
                    "snapshot_path": rec.snapshot_path,
                }
                self.store.log_event(
                    "TARGET_CONFIRMED",
                    self.state.value,
                    f"ALERT: {rec.target_id} [{rec.triage_priority} {rec.posture_label}] at ({rec.x_m:.2f}m, {rec.y_m:.2f}m) | Route: {rec.rescue_directions}",
                    rec.to_dict(),
                )
        elif tentative and self.cfg.planner.smart_confirm_enabled:
            self.tentative_sightings_count += len(tentative)
            best_tent = max(tentative, key=lambda c: c.mean_confidence)
            curr = self.current_cell()
            step_cell = self.grid.compute_ai_confirmation_step(curr, self.pose_est.pose.heading_deg, best_tent.bearing_deg)
            self.pending_ai_confirm_cell = step_cell

        return candidates

    def manual_scan_now(self) -> List[SightingCandidate]:
        burst_detections = []
        last_frame = None
        for f_idx in range(self.cfg.vision.scan_frames):
            frame, _, _ = self.frame_source.get_latest_frame()
            if frame is not None:
                last_frame = frame
                boxes = self.detector.detect(frame, frame_index=f_idx)
                burst_detections.append(boxes)
            if f_idx + 1 < self.cfg.vision.scan_frames:
                time.sleep(0.04)

        self.grid.raycast_observed_fov(self.pose_est.pose.x_m, self.pose_est.pose.y_m, self.pose_est.pose.heading_deg)
        if last_frame is not None and self.cfg.vision.hazard_detection_enabled:
            hazards = detect_ground_hazards(last_frame)
            if hazards:
                curr = self.current_cell()
                ahead = self.grid.cell_ahead(curr[0], curr[1], self.pose_est.pose.heading_deg)
                if self.grid.mark_discovered_hazard(ahead[0], ahead[1], current_cell=curr):
                    self.associator.refresh_all_rescue_paths()
                    self.store.save_targets_database(self.associator.targets)
                    self.store.log_event(
                        "HAZARD_DETECTED",
                        self.state.value,
                        f"Semantic ground hazard detected visually; marked cell {ahead} as HAZARD and updated A* routes",
                        {"cell": list(ahead), "hazards": hazards},
                    )
        candidates = self.tracker.evaluate_burst(burst_detections, self.pose_est.pose)
        confirmed = [c for c in candidates if c.confirmed]
        tentative = [c for c in candidates if c.tentative]

        if last_frame is not None and (confirmed or tentative):
            for cand in confirmed + tentative:
                tag = "confirmed" if cand.confirmed else "tentative"
                cand.snapshot_path = self.store.save_snapshot(
                    last_frame,
                    cand.boxes,
                    label_prefix=tag,
                    caption=f"LIVE SCAN {tag.upper()} conf={cand.mean_confidence:.2f} dist={cand.est_distance_m:.2f}m",
                )

        if confirmed:
            records = self.associator.associate_burst_candidates(confirmed, self.pose_est.pose)
            self.store.save_targets_database(self.associator.targets)
            self.latest_annotated_boxes = [b.to_dict() for c in confirmed for b in c.boxes]
            for rec in records:
                self.alert_seq += 1
                self.latest_alert = {
                    "alert_seq": self.alert_seq,
                    "timestamp": time.time(),
                    "target_id": rec.target_id,
                    "association_label": rec.association_label,
                    "triage_priority": rec.triage_priority,
                    "posture_label": rec.posture_label,
                    "x_m": rec.x_m,
                    "y_m": rec.y_m,
                    "col": rec.col,
                    "row": rec.row,
                    "confidence": rec.confidence,
                    "distance_from_home_m": rec.distance_from_home_m,
                    "rescue_path_distance_m": rec.rescue_path_distance_m,
                    "rescue_directions": rec.rescue_directions,
                    "path_from_home": rec.path_from_home,
                    "ibvs_turn_deg": rec.ibvs_turn_deg,
                    "snapshot_path": rec.snapshot_path,
                }
        self.save_artifacts()
        return candidates

    def execute_ibvs_alignment(self) -> Dict[str, Any]:
        if not self.associator.targets:
            return {"ok": False, "detail": "No targets available for IBVS alignment"}
        latest_t = self.associator.targets[-1]
        turn_deg = float(latest_t.ibvs_turn_deg)
        if abs(turn_deg) >= 2.0 and self.state in (MissionState.IDLE, MissionState.PAUSED, MissionState.RECOVERY_REQUIRED):
            st = self.robot.execute_move("turn", turn_deg)
            self.pose_est.update_from_turn(st.executed_deg)
            exec_deg = st.executed_deg
        else:
            exec_deg = turn_deg
        latest_t.ibvs_turn_deg = 0.0
        return {"ok": True, "target_id": latest_t.target_id, "commanded_deg": turn_deg, "executed_deg": exec_deg}

    def simulate_person_sighting(
        self,
        confidence: float = 0.88,
        box_height_px: int = 160,
        offset_x_px: int = 0,
        box_width_px: Optional[int] = None,
    ) -> List[SightingCandidate]:
        cx = 320 + offset_x_px
        cy = 240
        half_w = max(20, int((box_width_px * 0.5) if box_width_px is not None else (box_height_px * 0.32)))
        half_h = max(14, int(box_height_px * 0.5))
        b0 = DetectionBox(x1=cx - half_w, y1=cy - half_h, x2=cx + half_w, y2=cy + half_h, confidence=confidence, frame_index=0)
        b1 = DetectionBox(x1=cx - half_w + 1, y1=cy - half_h, x2=cx + half_w + 1, y2=cy + half_h, confidence=confidence, frame_index=1)
        burst = [[b0], [b1], []]
        candidates = self.tracker.evaluate_burst(burst, self.pose_est.pose)
        confirmed = [c for c in candidates if c.confirmed]
        tentative = [c for c in candidates if c.tentative]

        frame, _, _ = self.frame_source.get_latest_frame()
        if frame is not None:
            for cand in confirmed + tentative:
                tag = "confirmed" if cand.confirmed else "tentative"
                cand.snapshot_path = self.store.save_snapshot(
                    frame, cand.boxes, label_prefix=tag, caption=f"SIMULATED {tag.upper()} conf={cand.mean_confidence:.2f}"
                )

        if confirmed:
            records = self.associator.associate_burst_candidates(confirmed, self.pose_est.pose)
            self.store.save_targets_database(self.associator.targets)
            self.latest_annotated_boxes = [b.to_dict() for c in confirmed for b in c.boxes]
            for rec in records:
                self.alert_seq += 1
                self.latest_alert = {
                    "alert_seq": self.alert_seq,
                    "timestamp": time.time(),
                    "target_id": rec.target_id,
                    "association_label": rec.association_label,
                    "triage_priority": rec.triage_priority,
                    "posture_label": rec.posture_label,
                    "x_m": rec.x_m,
                    "y_m": rec.y_m,
                    "col": rec.col,
                    "row": rec.row,
                    "confidence": rec.confidence,
                    "distance_from_home_m": rec.distance_from_home_m,
                    "rescue_path_distance_m": rec.rescue_path_distance_m,
                    "rescue_directions": rec.rescue_directions,
                    "path_from_home": rec.path_from_home,
                    "ibvs_turn_deg": rec.ibvs_turn_deg,
                    "snapshot_path": rec.snapshot_path,
                }
        self.save_artifacts()
        return candidates

    def _initiate_return_home(self, reason: str) -> None:
        self.returning_home_mode = True
        self.transition_to(MissionState.RETURNING_HOME, reason)

    def pause(self) -> None:
        self.robot.stop()
        if self.state not in (MissionState.PAUSED, MissionState.IDLE):
            self.transition_to(MissionState.PAUSED, "operator_pause")

    def resume(self, background: bool = True) -> None:
        if self.state == MissionState.PAUSED:
            next_st = MissionState.RETURNING_HOME if self.returning_home_mode else MissionState.PLANNING
            self.transition_to(next_st, "operator_resume")
            if background:
                self._worker_thread = threading.Thread(target=self._mission_loop, daemon=True)
                self._worker_thread.start()

    def abort(self) -> None:
        self._stop_flag.set()
        try:
            self.robot.stop()
        except Exception:
            pass
        self.transition_to(MissionState.MISSION_ABORTED, "operator_abort")
        self.save_artifacts()

    def trigger_return_home(self, background: bool = True) -> None:
        self._initiate_return_home("operator_requested")
        if background and (self._worker_thread is None or not self._worker_thread.is_alive()):
            self._worker_thread = threading.Thread(target=self._mission_loop, daemon=True)
            self._worker_thread.start()

    def e_stop(self) -> RobotStatus:
        self._stop_flag.set()
        st = self.robot.estop()
        self.state = MissionState.MISSION_ABORTED
        self.save_artifacts()
        return st

    def reset_estop(self) -> RobotStatus:
        st = self.robot.reset_estop()
        self.transition_to(MissionState.IDLE, "estop_reset")
        return st

    def goto_cell(self, col: int, row: int, background: bool = True) -> Dict[str, Any]:
        """Click-to-Navigate (2D Nav Goal): Compute collision-free A* path to (col, row) and drive there."""
        if not self.grid.is_traversable(col, row):
            return {"ok": False, "detail": f"Cell ({col},{row}) is blocked or out of bounds."}
        curr = self.current_cell()
        path = self.grid.astar_path(curr, (col, row))
        if path is None:
            return {"ok": False, "detail": f"No collision-free A* path from {curr} to ({col},{row})."}

        self.goto_goal = (col, row)
        self.planned_path = path
        self._stop_flag.clear()
        self.store.log_event("NAV_GOAL_SET", self.state.value, f"Click-to-Navigate A* goal set to ({col}, {row}) ({len(path)-1} steps)")

        if len(path) <= 1:
            self._perform_scan_at_current_cell(reason="nav_goal_arrived")
            self.transition_to(MissionState.IDLE, "nav_goal_reached")
            return {"ok": True, "path": [list(p) for p in path]}

        def _goto_worker() -> None:
            try:
                self.transition_to(MissionState.MOVING_TO_SCAN_POINT, f"goto_{col}_{row}")
                while not self._stop_flag.is_set():
                    c_now = self.current_cell()
                    if c_now == (col, row):
                        break
                    p_now = self.grid.astar_path(c_now, (col, row))
                    if not p_now or len(p_now) < 2:
                        break
                    self.planned_path = p_now
                    ok = self._execute_step_to_neighbor(c_now, p_now[1])
                    if not ok:
                        break
                self._perform_scan_at_current_cell(reason="nav_goal_arrived")
                self.transition_to(MissionState.IDLE, "nav_goal_reached")
            except Exception as exc:
                self.store.log_event("NAV_GOAL_ERROR", self.state.value, str(exc))
                self.state = MissionState.IDLE
            finally:
                self.goto_goal = None

        if background:
            self._worker_thread = threading.Thread(target=_goto_worker, name="GotoNavWorker", daemon=True)
            self._worker_thread.start()
        else:
            _goto_worker()
        return {"ok": True, "path": [list(p) for p in path]}

    def manual_move(self, move_type: str, value: float) -> RobotStatus:
        if move_type == "stop":
            st = self.robot.stop()
            self.store.log_event("MANUAL_STOP", self.state.value, "Manual stop command sent")
            return st
        st = self.robot.execute_move(move_type, value)
        if move_type in ("forward", "backward"):
            self.pose_est.update_from_translation(st.executed_cm, direction=move_type)
            self.grid.mark_visited(*self.current_cell())
        elif move_type == "turn":
            self.pose_est.update_from_turn(st.executed_deg)
        self.store.log_event(
            "MANUAL_MOVE",
            self.state.value,
            f"Manual {move_type} {value:.1f} -> state={st.state}, us={st.ultrasonic_cm:.1f}cm",
            {"move_type": move_type, "value": value, "executed_cm": st.executed_cm, "executed_deg": st.executed_deg},
        )
        return st

    def save_artifacts(self) -> MissionReport:
        self.store.save_coverage_map(self.grid, self.pose_est.trail, self.planned_path)
        cov = self.grid.compute_coverage_stats()
        now = time.time()
        rep = MissionReport(
            run_id=self.store.run_id,
            evidence_mode=self.evidence_mode.value,
            final_state=self.state.value,
            started_at=self.started_at,
            ended_at=self.ended_at if self.ended_at > 0 else now,
            elapsed_s=max(0.0, (self.ended_at if self.ended_at > 0 else now) - self.started_at),
            step_count=self.step_count,
            coverage=cov,
            final_pose=self.pose_est.pose,
            targets=list(self.associator.targets),
            tentative_sightings_count=self.tentative_sightings_count,
            smart_confirmations_triggered=self.smart_confirmations_triggered,
            obstacles_discovered=len(self.grid.discovered_blocked),
            fault_count=self.fault_count,
            recovery_count=self.recovery_count,
            home_reached_estimated=self.pose_est.is_home_reached_estimated(),
            home_reached_operator_verified=self.pose_est.home_reached_operator_verified,
            home_operator_offset_cm=self.pose_est.home_operator_offset_cm,
            honest_limitations_note="Dual-layer boundary autonomous reconnaissance rover with Bayesian log-odds occupancy mapping.",
        )
        self.store.save_report(rep)
        return rep

    def get_telemetry_snapshot(self) -> Dict[str, Any]:
        if self.evidence_mode == EvidenceMode.REAL_HARDWARE and self.state in (MissionState.IDLE, MissionState.PAUSED):
            if hasattr(self.robot, "refresh_status_if_stale"):
                try:
                    self.robot.refresh_status_if_stale(max_age_s=1.8)
                except Exception:
                    pass
        cov = self.grid.compute_coverage_stats()
        return {
            "run_id": self.store.run_id,
            "evidence_mode": self.evidence_mode.value,
            "state": self.state.value,
            "step_count": self.step_count,
            "elapsed_s": round(max(0.0, time.time() - self.started_at), 1) if self.started_at > 0 else 0.0,
            "pose": self.pose_est.pose.to_dict(),
            "robot_status": self.robot.last_status.to_dict() if hasattr(self.robot, "last_status") else {},
            "coverage": cov.to_dict(),
            "grid": self.grid.to_dict(),
            "trail": [list(pt) for pt in self.pose_est.trail[-200:]],
            "planned_path": [list(pt) for pt in self.planned_path],
            "targets": [t.to_dict() for t in self.associator.targets],
            "latest_alert": self.latest_alert,
            "rescue_tour": self.associator.compute_multi_target_rescue_tour(),
            "visual_odometry": self.last_vo_status,
            "goto_goal": list(self.goto_goal) if self.goto_goal else None,
            "detector_device": self.detector.device_name,
            "camera_fps": getattr(self.frame_source, "fps", 0.0),
            "events": [e.to_dict() for e in self.store.events[-40:]],
        }
