"""Unit tests for Boustrophedon serpentine coverage, nearest-cell maze planner, A* routing, and AI target confirmation."""
from __future__ import annotations

from rover.config import ArenaConfig, PlannerConfig
from rover.grid import ArenaGrid


def test_serpentine_boustrophedon_order() -> None:
    cfg = ArenaConfig(width_m=1.2, height_m=0.9, cell_size_m=0.30)  # 4 cols x 3 rows
    grid = ArenaGrid(cfg, PlannerConfig(coverage_mode="serpentine", lane_stride=1, scan_spacing=2))

    # Row 0: left-to-right (0,0)..(3,0), Row 1: right-to-left (3,1)..(0,1), Row 2: left-to-right (0,2)..(3,2)
    assert grid.serpentine_order[:4] == [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert grid.serpentine_order[4:8] == [(3, 1), (2, 1), (1, 1), (0, 1)]
    assert grid.serpentine_order[8:12] == [(0, 2), (1, 2), (2, 2), (3, 2)]


def test_astar_detours_around_obstacle() -> None:
    cfg = ArenaConfig(
        width_m=1.5,
        height_m=0.9,
        cell_size_m=0.30,
        configured_obstacles=[(1, 0), (1, 1)],
    )
    grid = ArenaGrid(cfg)
    path = grid.astar_path((0, 0), (2, 0))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (2, 0)
    assert (1, 0) not in path
    assert (1, 1) not in path


def test_nearest_coverage_mode_and_ai_confirmation_priority() -> None:
    cfg = ArenaConfig(width_m=1.5, height_m=1.2, cell_size_m=0.30)
    p_cfg = PlannerConfig(coverage_mode="nearest", smart_confirm_enabled=True)
    grid = ArenaGrid(cfg, p_cfg)
    grid.mark_visited(0, 0)

    # Without AI confirmation pending, selects nearest unvisited cell
    target, path, is_confirm = grid.next_coverage_target((0, 0))
    assert target in ((1, 0), (0, 1))
    assert path is not None
    assert is_confirm is False

    # With AI confirmation step pending at (2, 1), prioritizes (2, 1)
    confirm_cell = grid.compute_ai_confirmation_step((1, 1), rover_heading_deg=0.0, relative_bearing_deg=5.0)
    assert confirm_cell == (2, 1)
    t2, p2, is_c2 = grid.next_coverage_target((1, 1), pending_ai_confirm_cell=confirm_cell)
    assert t2 == (2, 1)
    assert p2 == [(1, 1), (2, 1)]
    assert is_c2 is True
