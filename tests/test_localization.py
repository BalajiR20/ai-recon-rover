"""Unit tests for PoseEstimator executed-amount kinematics, arena clamping, v5.0 uncertainty model, and gated wall-snap."""
from __future__ import annotations

import pytest
from rover.config import ArenaConfig, LocalizationConfig
from rover.grid import ArenaGrid
from rover.localization import PoseEstimator, angle_diff_deg, normalize_angle_deg


def test_executed_distance_update_and_clamping() -> None:
    arena_cfg = ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30)
    loc_cfg = LocalizationConfig(k_dist=0.03, k_turn_deg=2.5, sigma_pos_max_m=0.80, sigma_heading_max_deg=60.0)
    grid = ArenaGrid(arena_cfg)
    est = PoseEstimator(grid, loc_cfg, arena_cfg)

    x0, y0 = est.pose.x_m, est.pose.y_m
    # Commanded 30 cm, but firmware aborted at 14.5 cm -> pose must update by 0.145 m
    est.update_from_translation(executed_cm=14.5, direction="forward")
    assert est.pose.x_m == pytest.approx(x0 + 0.145, abs=1e-4)
    assert est.pose.y_m == pytest.approx(y0, abs=1e-4)
    assert est.pose.sigma_pos_m > 0.05

    # Huge forward move cannot cross physical East wall
    est.update_from_translation(executed_cm=500.0, direction="forward")
    assert est.pose.x_m <= arena_cfg.width_m - grid.clamp_margin_x_m + 1e-4


def test_v5_heading_uncertainty_avoids_four_turn_lockup() -> None:
    arena_cfg = ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30)
    loc_cfg = LocalizationConfig(k_turn_deg=2.5, sigma_heading_max_deg=60.0)
    grid = ArenaGrid(arena_cfg)
    est = PoseEstimator(grid, loc_cfg, arena_cfg)

    for _ in range(8):
        est.update_from_turn(executed_deg=90.0)
    exceeded, _ = est.check_uncertainty_exceeded()
    assert exceeded is False
    assert est.pose.sigma_heading_deg == pytest.approx(2.0 + 8 * 2.5, abs=1e-4)


def test_operator_recovery_and_gated_wall_snap() -> None:
    arena_cfg = ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30)
    loc_cfg = LocalizationConfig(enable_wall_snap=True, wall_snap_max_correction_m=0.25)
    grid = ArenaGrid(arena_cfg)
    est = PoseEstimator(grid, loc_cfg, arena_cfg)

    # Place rover near East wall facing 0 deg
    est.confirm_operator_recovery(col=8, row=2, heading_deg=0.0)
    assert est.pose.sigma_pos_m == pytest.approx(0.05)
    assert est.pose.sigma_heading_deg == pytest.approx(2.0)

    # Expected distance from cell 8 center (2.55 m) to East wall (3.00 m) is 45 cm.
    # Valid ultrasonic reading of 42.0 cm (correction = 0.03 m < 0.25 m, spread = 0.4 cm < 1.0 cm)
    ok, reason = est.try_wall_snap([42.0, 42.2, 41.8])
    assert ok is True
    assert reason == "snapped_east"
    assert est.pose.x_m == pytest.approx(3.0 - 0.42, abs=1e-3)

    # Outlier reading (150 cm -> > 0.25 m jump) is rejected
    ok2, reason2 = est.try_wall_snap([150.0, 150.1, 149.9])
    assert ok2 is False
    assert "outlier_correction" in reason2


def test_angle_normalization_helpers() -> None:
    assert normalize_angle_deg(185.0) == pytest.approx(-175.0)
    assert normalize_angle_deg(-185.0) == pytest.approx(175.0)
    assert angle_diff_deg(10.0, 350.0) == pytest.approx(20.0)
    assert angle_diff_deg(350.0, 10.0) == pytest.approx(-20.0)
