"""Dead-reckoning, pose estimation, and uncertainty propagation for AI Recon Rover v5.5."""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from rover.config import ArenaConfig, LocalizationConfig
from rover.grid import ArenaGrid
from rover.models import PoseSource, PoseState


def normalize_angle_deg(ang: float) -> float:
    while ang > 180.0:
        ang -= 360.0
    while ang <= -180.0:
        ang += 360.0
    return ang


def angle_diff_deg(a: float, b: float) -> float:
    return normalize_angle_deg(a - b)


class PoseEstimator:
    def __init__(
        self,
        grid: ArenaGrid,
        loc_cfg: Optional[LocalizationConfig] = None,
        arena_cfg: Optional[ArenaConfig] = None,
        is_simulated: bool = False,
    ) -> None:
        self.grid = grid
        self.loc_cfg = loc_cfg or LocalizationConfig()
        self.arena_cfg = arena_cfg or ArenaConfig()
        self.is_simulated = is_simulated

        hx, hy = grid.cell_to_world(self.arena_cfg.home_col, self.arena_cfg.home_row)
        self.pose = PoseState(
            x_m=hx,
            y_m=hy,
            heading_deg=normalize_angle_deg(self.arena_cfg.home_heading_deg),
            sigma_pos_m=0.05,
            sigma_heading_deg=2.0,
            pose_source=PoseSource.DEAD_RECKONING.value,
            timestamp=time.time(),
            is_simulated=is_simulated,
        )
        self.trail: list[Tuple[float, float]] = [(self.pose.x_m, self.pose.y_m)]
        self.home_reached_operator_verified: Optional[bool] = None
        self.home_operator_offset_cm: Optional[float] = None

    def update_from_translation(self, executed_cm: float, direction: str = "forward") -> None:
        dist_m = (executed_cm / 100.0) if direction == "forward" else (-executed_cm / 100.0)
        rad = math.radians(self.pose.heading_deg)
        new_x = self.pose.x_m + dist_m * math.cos(rad)
        new_y = self.pose.y_m + dist_m * math.sin(rad)

        min_x = self.grid.clamp_margin_x_m
        max_x = self.arena_cfg.width_m - self.grid.clamp_margin_x_m
        min_y = self.grid.clamp_margin_y_m
        max_y = self.arena_cfg.height_m - self.grid.clamp_margin_y_m

        self.pose.x_m = round(max(min_x, min(max_x, new_x)), 4)
        self.pose.y_m = round(max(min_y, min(max_y, new_y)), 4)
        self.pose.sigma_pos_m = round(self.pose.sigma_pos_m + self.loc_cfg.k_dist * abs(dist_m), 4)
        self.pose.timestamp = time.time()
        self.trail.append((self.pose.x_m, self.pose.y_m))

    def update_from_turn(self, executed_deg: float, gyro_active: bool = False) -> None:
        self.pose.heading_deg = round(normalize_angle_deg(self.pose.heading_deg + executed_deg), 2)
        rate = self.loc_cfg.k_turn_gyro_deg if gyro_active else self.loc_cfg.k_turn_deg
        inc = (abs(executed_deg) / 90.0) * rate
        self.pose.sigma_heading_deg = round(self.pose.sigma_heading_deg + inc, 2)
        self.pose.timestamp = time.time()

    def check_uncertainty_exceeded(self) -> Tuple[bool, str]:
        if self.pose.sigma_pos_m > self.loc_cfg.sigma_pos_max_m:
            return True, f"Position uncertainty {self.pose.sigma_pos_m:.2f}m > max {self.loc_cfg.sigma_pos_max_m:.2f}m"
        if self.pose.sigma_heading_deg > self.loc_cfg.sigma_heading_max_deg:
            return True, f"Heading uncertainty {self.pose.sigma_heading_deg:.1f}° > max {self.loc_cfg.sigma_heading_max_deg:.1f}°"
        return False, ""

    def confirm_operator_recovery(self, col: int, row: int, heading_deg: float) -> None:
        wx, wy = self.grid.cell_to_world(col, row)
        self.pose.x_m = wx
        self.pose.y_m = wy
        self.pose.heading_deg = normalize_angle_deg(heading_deg)
        self.pose.sigma_pos_m = 0.05
        self.pose.sigma_heading_deg = 2.0
        self.pose.pose_source = PoseSource.OPERATOR_CONFIRMED.value
        self.pose.timestamp = time.time()
        self.trail.append((wx, wy))

    def acknowledge_drift_continue(self) -> None:
        self.pose.sigma_pos_m = min(self.pose.sigma_pos_m, self.loc_cfg.sigma_pos_max_m * 0.85)
        self.pose.sigma_heading_deg = min(self.pose.sigma_heading_deg, self.loc_cfg.sigma_heading_max_deg * 0.85)

    def is_home_reached_estimated(self) -> bool:
        hx, hy = self.grid.cell_to_world(self.arena_cfg.home_col, self.arena_cfg.home_row)
        dist = math.hypot(self.pose.x_m - hx, self.pose.y_m - hy)
        return dist <= self.loc_cfg.home_tolerance_m

    def record_operator_home_verification(self, verified: bool, offset_cm: Optional[float] = None) -> None:
        self.home_reached_operator_verified = verified
        self.home_operator_offset_cm = offset_cm
