"""Unit tests for ArenaGrid geometry, configured vs discovered obstacles, and ray-cast observed coverage."""
from __future__ import annotations

import pytest
from rover.config import ArenaConfig, CameraConfig, PlannerConfig
from rover.grid import ArenaGrid
from rover.models import CellStatus


def test_grid_dimensions_and_margins() -> None:
    cfg = ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30)
    grid = ArenaGrid(cfg)
    assert grid.cols == 10
    assert grid.rows == 6
    assert grid.remainder_x_m == pytest.approx(0.0, abs=1e-4)
    assert grid.remainder_y_m == pytest.approx(0.20, abs=1e-4)

    stats = grid.compute_coverage_stats()
    assert stats.cols == 10
    assert stats.rows == 6
    assert stats.total_cells == 60
    assert stats.eligible_cells == 60
    assert stats.visited_coverage_pct == 0.0
    assert stats.observed_coverage_pct == 0.0


def test_configured_vs_discovered_obstacles() -> None:
    cfg = ArenaConfig(width_m=1.5, height_m=1.5, cell_size_m=0.30, configured_obstacles=[(2, 2)])
    grid = ArenaGrid(cfg)
    assert grid.is_blocked(2, 2)
    assert (2, 2) in grid.configured_blocked
    assert (2, 2) not in grid.discovered_blocked

    grid.mark_discovered_obstacle(1, 2, current_cell=(0, 0))
    assert grid.is_blocked(1, 2)
    assert (1, 2) in grid.discovered_blocked
    assert (1, 2) not in grid.configured_blocked

    stats = grid.compute_coverage_stats()
    assert stats.configured_blocked_cells == 1
    assert stats.discovered_blocked_cells == 1
    assert stats.eligible_cells == 25 - 2


def test_unreachable_cells_honest_coverage() -> None:
    # 4x3 grid where (3, 0) is walled off by (2, 0) and (3, 1)
    cfg = ArenaConfig(
        width_m=1.2,
        height_m=0.9,
        cell_size_m=0.30,
        home_col=0,
        home_row=0,
        configured_obstacles=[(2, 0), (3, 1)],
    )
    grid = ArenaGrid(cfg)
    assert grid.status_grid[(3, 0)] == CellStatus.UNREACHABLE

    # Visit all reachable cells
    for c in range(grid.cols):
        for r in range(grid.rows):
            if grid.status_grid[(c, r)] not in (CellStatus.BLOCKED, CellStatus.UNREACHABLE):
                grid.mark_visited(c, r)

    stats = grid.compute_coverage_stats()
    # (3, 0) is still UNREACHABLE and eligible -> visited coverage must be < 100%
    assert stats.unreachable_cells == 1
    assert stats.visited_coverage_pct < 100.0


def test_raycast_observed_fov_stops_at_obstacle() -> None:
    cfg = ArenaConfig(
        width_m=3.0,
        height_m=2.0,
        cell_size_m=0.30,
        home_col=0,
        home_row=2,
        configured_obstacles=[(3, 2)],
    )
    cam_cfg = CameraConfig(hfov_deg=60.0, detect_range_m=2.0)
    grid = ArenaGrid(cfg, camera_cfg=cam_cfg)
    rx, ry = grid.cell_to_world(0, 2)
    observed = grid.raycast_observed_fov(rx, ry, rover_heading_deg=0.0)

    # Cells directly in front before the obstacle should be observed
    assert (1, 2) in observed
    assert (2, 2) in observed
    # Obstacle cell (3, 2) and shadowed cell (4, 2) directly behind it should not be observed
    assert (3, 2) not in observed
    assert (4, 2) not in observed
