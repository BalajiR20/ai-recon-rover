"""Typed data models and enumerations for AI-Assisted Search & Reconnaissance Rover v5.5."""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class CellStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    FREE = "FREE"
    VISITED = "VISITED"
    BLOCKED = "BLOCKED"
    HAZARD = "HAZARD"
    UNCERTAIN = "UNCERTAIN"
    UNREACHABLE = "UNREACHABLE"


class MissionState(str, Enum):
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    PLANNING = "PLANNING"
    MOVING_TO_SCAN_POINT = "MOVING_TO_SCAN_POINT"
    SCANNING = "SCANNING"
    REPLANNING = "REPLANNING"
    RETURNING_HOME = "RETURNING_HOME"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"
    PAUSED = "PAUSED"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_ABORTED = "MISSION_ABORTED"
    FAULT = "FAULT"


class EvidenceMode(str, Enum):
    MOCK = "MOCK"
    RECORDED_VIDEO = "RECORDED_VIDEO"
    REAL_HARDWARE = "REAL_HARDWARE"


class PoseSource(str, Enum):
    DEAD_RECKONING = "dead_reckoning"
    GYRO_ASSISTED = "gyro_assisted"
    OPERATOR_CONFIRMED = "operator_confirmed"


class AssociationLabel(str, Enum):
    NEW_TARGET = "NEW_TARGET"
    LIKELY_SAME_TARGET = "LIKELY_SAME_TARGET"
    POSSIBLE_MATCH = "POSSIBLE_MATCH"
    UNCERTAIN_REVIEW = "UNCERTAIN_REVIEW"


@dataclass
class PoseState:
    x_m: float
    y_m: float
    heading_deg: float
    sigma_pos_m: float = 0.05
    sigma_heading_deg: float = 2.0
    pose_source: str = PoseSource.DEAD_RECKONING.value
    timestamp: float = field(default_factory=time.time)
    is_simulated: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MoveCommand:
    id: int
    type: str  # "forward", "backward", "turn"
    value: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MoveAck:
    id: int
    accepted: bool
    state: str
    duplicate: bool = False
    reason: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class RobotStatus:
    boot_id: int = 0
    last_id: int = 0
    state: str = "idle"
    last_executed_state: str = "none"
    elapsed_ms: int = 0
    executed_cm: float = 0.0
    executed_deg: float = 0.0
    executed_source: str = "time"
    ultrasonic_cm: float = -1.0
    rssi: int = -50
    uptime_ms: int = 0
    fw_version: str = "5.5.0"
    connected: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class DetectionBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int = 0
    frame_index: int = 0
    timestamp: float = field(default_factory=time.time)
    keypoints: List[List[float]] = field(default_factory=list)

    @property
    def width_px(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height_px(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def center_x_px(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def center_y_px(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["width_px"] = self.width_px
        d["height_px"] = self.height_px
        return d


@dataclass
class SightingCandidate:
    boxes: List[DetectionBox]
    mean_confidence: float
    max_height_px: float
    frames_seen: int
    confirmed: bool
    tentative: bool
    bearing_deg: float
    est_distance_m: float
    est_x_m: float
    est_y_m: float
    est_col: int
    est_row: int
    snapshot_path: Optional[str] = None
    posture_label: str = "STANDING"
    triage_priority: str = "P3-AMBULATORY"
    aspect_ratio: float = 0.45
    ibvs_turn_deg: float = 0.0
    is_waving: bool = False
    torso_angle_deg: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "boxes": [b.to_dict() for b in self.boxes],
            "mean_confidence": round(self.mean_confidence, 3),
            "max_height_px": round(self.max_height_px, 1),
            "frames_seen": self.frames_seen,
            "confirmed": self.confirmed,
            "tentative": self.tentative,
            "bearing_deg": round(self.bearing_deg, 2),
            "est_distance_m": round(self.est_distance_m, 2),
            "est_x_m": round(self.est_x_m, 3),
            "est_y_m": round(self.est_y_m, 3),
            "est_col": self.est_col,
            "est_row": self.est_row,
            "snapshot_path": self.snapshot_path,
            "posture_label": self.posture_label,
            "triage_priority": self.triage_priority,
            "aspect_ratio": round(self.aspect_ratio, 2),
            "ibvs_turn_deg": round(self.ibvs_turn_deg, 2),
            "is_waving": self.is_waving,
            "torso_angle_deg": round(self.torso_angle_deg, 1),
        }


@dataclass
class TargetRecord:
    target_id: str
    association_label: str
    x_m: float
    y_m: float
    col: int
    row: int
    confidence: float
    frames_seen: int
    sightings_count: int
    first_seen_ts: float
    last_seen_ts: float
    rover_x_m: float
    rover_y_m: float
    rover_heading_deg: float
    sigma_pos_m: float
    snapshot_path: Optional[str] = None
    distance_from_home_m: float = 0.0
    rescue_path_distance_m: float = 0.0
    path_from_home: List[List[int]] = field(default_factory=list)
    rescue_directions: str = ""
    posture_label: str = "STANDING"
    triage_priority: str = "P3-AMBULATORY"
    aspect_ratio: float = 0.45
    ibvs_turn_deg: float = 0.0
    is_waving: bool = False
    torso_angle_deg: float = 0.0
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class CoverageStats:
    cols: int
    rows: int
    total_cells: int
    configured_blocked_cells: int
    discovered_blocked_cells: int
    eligible_cells: int
    visited_cells: int
    observed_cells: int
    unknown_cells: int
    free_cells: int
    uncertain_cells: int
    unreachable_cells: int
    visited_coverage_pct: float
    observed_coverage_pct: float
    margin_x_m: float
    margin_y_m: float
    shannon_entropy_bits: float = 0.0
    entropy_reduction_pct: float = 0.0
    frontier_cells_count: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PreflightCheckItem:
    name: str
    passed: bool
    detail: str
    fix_instruction: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PreflightResult:
    passed: bool
    checks: List[PreflightCheckItem]
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "passed": self.passed,
            "timestamp": self.timestamp,
            "checks": [c.to_dict() for c in self.checks],
        }


@dataclass
class MissionEvent:
    timestamp: float
    event_type: str
    state: str
    message: str
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionReport:
    run_id: str
    evidence_mode: str
    final_state: str
    started_at: float
    ended_at: float
    elapsed_s: float
    step_count: int
    coverage: CoverageStats
    final_pose: PoseState
    targets: List[TargetRecord]
    tentative_sightings_count: int
    smart_confirmations_triggered: int
    obstacles_discovered: int
    fault_count: int
    recovery_count: int
    home_reached_estimated: bool
    home_reached_operator_verified: Optional[bool]
    home_operator_offset_cm: Optional[float]
    honest_limitations_note: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "run_id": self.run_id,
            "evidence_mode": self.evidence_mode,
            "final_state": self.final_state,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "elapsed_s": round(self.elapsed_s, 2),
            "step_count": self.step_count,
            "coverage": self.coverage.to_dict(),
            "final_pose": self.final_pose.to_dict(),
            "targets": [t.to_dict() for t in self.targets],
            "tentative_sightings_count": self.tentative_sightings_count,
            "smart_confirmations_triggered": self.smart_confirmations_triggered,
            "obstacles_discovered": self.obstacles_discovered,
            "fault_count": self.fault_count,
            "recovery_count": self.recovery_count,
            "home_reached_estimated": self.home_reached_estimated,
            "home_reached_operator_verified": self.home_reached_operator_verified,
            "home_operator_offset_cm": self.home_operator_offset_cm,
            "honest_limitations_note": self.honest_limitations_note,
        }
