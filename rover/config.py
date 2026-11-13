"""Configuration loading, validation, and serialization for AI Recon Rover v5.6."""
from __future__ import annotations

import pathlib
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import yaml


@dataclass
class ArenaConfig:
    width_m: float = 2.0
    height_m: float = 1.5
    cell_size_m: float = 0.30
    home_col: int = 0
    home_row: int = 0
    home_heading_deg: float = 0.0
    configured_obstacles: List[Tuple[int, int]] = field(default_factory=list)

    def validate(self) -> None:
        if self.width_m <= 0 or self.height_m <= 0 or self.cell_size_m <= 0:
            raise ValueError("Arena dimensions and cell size must be positive.")


@dataclass
class PlannerConfig:
    coverage_mode: str = "serpentine"  # "serpentine", "frontier_ig", "nearest"
    lane_stride: int = 1
    scan_spacing: int = 2
    max_coalesced_cells: int = 2
    smart_confirm_enabled: bool = True
    max_confirm_steps_per_target: int = 1
    turn_scan_enabled: bool = False
    fov_assist_visit: bool = True


@dataclass
class LocalizationConfig:
    k_dist: float = 0.03
    k_turn_deg: float = 2.5
    k_turn_gyro_deg: float = 1.0
    sigma_pos_max_m: float = 0.80
    sigma_heading_max_deg: float = 60.0
    uncertainty_policy: str = "warn_only"
    enable_wall_snap: bool = False
    wall_snap_max_correction_m: float = 0.25
    wall_snap_heading_tol_deg: float = 8.0
    wall_snap_spread_max_cm: float = 1.0
    home_tolerance_m: float = 0.40


@dataclass
class RobotConfig:
    devkit_url: str = "http://192.168.43.50:80"
    mock_url: str = "http://127.0.0.1:8081"
    idle_heartbeat_hz: float = 2.0
    active_poll_hz: float = 4.0
    watchdog_ms: int = 1500
    http_timeout_s: float = 1.5
    max_move_cm: float = 100.0
    max_turn_deg: float = 180.0
    stop_distance_cm: float = 20.0
    bumper_backup_cm: float = 10.0
    use_imu: bool = False
    enable_bumper: bool = False


@dataclass
class CameraConfig:
    stream_url: str = "http://192.168.43.1:4747/video"
    reconnect_cooldown_s: float = 1.0
    stall_timeout_s: float = 2.5
    max_frame_age_s: float = 1.5
    width: int = 640
    height: int = 480
    hfov_deg: float = 60.0
    detect_range_m: float = 2.0


@dataclass
class VisionConfig:
    model_path: str = "models/yolov8n-pose.pt"
    fallback_model_path: str = "models/yolov8n.pt"
    imgsz: int = 416
    device: str = "auto"
    detect_while_moving: bool = False
    settle_s: float = 0.25
    camera_latency_s: float = 0.20
    discard_frames: int = 1
    scan_frames: int = 3
    min_confirm_frames: int = 2
    confirm_conf_min: float = 0.35
    tentative_conf_min: float = 0.22
    min_box_height_px: int = 36
    tentative_min_box_height_px: int = 25
    iou_match_threshold: float = 0.25
    focal_scale_px: float = 380.0
    assumed_person_height_m: float = 1.65
    hazard_detection_enabled: bool = True
    optical_flow_enabled: bool = True
    vo_heading_correction_enabled: bool = True
    vo_auto_trim_enabled: bool = True


@dataclass
class AssociationConfig:
    likely_same_dist_m: float = 0.60
    possible_match_dist_m: float = 1.10
    base_gate_m: float = 0.85


@dataclass
class MissionConfig:
    max_mission_time_s: float = 480.0
    max_steps_multiplier: int = 3
    max_cell_revisits: int = 4
    max_steps_without_progress: int = 12
    max_consecutive_invalid_us: int = 5
    warn_consecutive_invalid_us: int = 3


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000
    ws_telemetry_hz: float = 5.0
    runs_dir: str = "runs"
    min_free_disk_mb: float = 50.0


@dataclass
class CalibrationData:
    cm_per_s: float = 25.0
    deg_per_s: float = 90.0
    deg_per_s_left: float = 0.0
    deg_per_s_right: float = 0.0
    speed_pwm: int = 170
    turn_pwm: int = 165
    min_pwm: int = 110
    max_pwm: int = 240
    left_trim: float = 1.0
    right_trim: float = 1.0
    motor_invert_left: bool = False
    motor_invert_right: bool = False
    stop_distance_cm: float = 20.0
    watchdog_ms: int = 1500
    calibrated_at: float = 0.0
    notes: str = ""

    def effective_deg_per_s(self, turn_deg: float) -> float:
        if turn_deg >= 0.0 and self.deg_per_s_left > 1.0:
            return self.deg_per_s_left
        if turn_deg < 0.0 and self.deg_per_s_right > 1.0:
            return self.deg_per_s_right
        return max(1.0, self.deg_per_s)

    def validate(self) -> None:
        if self.cm_per_s <= 0 or self.deg_per_s <= 0:
            raise ValueError("Calibration rates must be positive.")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AppConfig:
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    association: AssociationConfig = field(default_factory=AssociationConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    server: ServerConfig = field(default_factory=ServerConfig)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["arena"]["configured_obstacles"] = [list(o) for o in self.arena.configured_obstacles]
        return d


def _filter_dataclass_kwargs(cls, raw_dict: Dict[str, Any]) -> Dict[str, Any]:
    valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
    return {k: v for k, v in raw_dict.items() if k in valid_keys}


def load_config(path: str | pathlib.Path = "config.yaml") -> AppConfig:
    p = pathlib.Path(path)
    if not p.is_file():
        cfg = AppConfig()
        save_config(cfg, p)
        return cfg
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    obs = []
    for item in data.get("arena", {}).get("configured_obstacles", []):
        if isinstance(item, (list, tuple)) and len(item) == 2:
            obs.append((int(item[0]), int(item[1])))

    arena_kwargs = _filter_dataclass_kwargs(
        ArenaConfig, {k: v for k, v in data.get("arena", {}).items() if k != "configured_obstacles"}
    )
    return AppConfig(
        arena=ArenaConfig(**arena_kwargs, configured_obstacles=obs),
        planner=PlannerConfig(**_filter_dataclass_kwargs(PlannerConfig, data.get("planner", {}))),
        localization=LocalizationConfig(**_filter_dataclass_kwargs(LocalizationConfig, data.get("localization", {}))),
        robot=RobotConfig(**_filter_dataclass_kwargs(RobotConfig, data.get("robot", {}))),
        camera=CameraConfig(**_filter_dataclass_kwargs(CameraConfig, data.get("camera", {}))),
        vision=VisionConfig(**_filter_dataclass_kwargs(VisionConfig, data.get("vision", {}))),
        association=AssociationConfig(**_filter_dataclass_kwargs(AssociationConfig, data.get("association", {}))),
        mission=MissionConfig(**_filter_dataclass_kwargs(MissionConfig, data.get("mission", {}))),
        server=ServerConfig(**_filter_dataclass_kwargs(ServerConfig, data.get("server", {}))),
    )


def save_config(cfg: AppConfig, path: str | pathlib.Path = "config.yaml") -> None:
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg.to_dict(), f, sort_keys=False)


def load_calibration(path: str | pathlib.Path = "calibration.json") -> CalibrationData:
    import json
    p = pathlib.Path(path)
    if not p.is_file():
        cal = CalibrationData()
        save_calibration(cal, p)
        return cal
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return CalibrationData(**_filter_dataclass_kwargs(CalibrationData, data))


def save_calibration(cal: CalibrationData, path: str | pathlib.Path = "calibration.json") -> None:
    import json
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        json.dump(cal.to_dict(), f, indent=2)
