"""Bounded arena grid, Boustrophedon / Frontier Information-Gain planner, A* routing, and Bayesian Log-Odds."""
from __future__ import annotations

import heapq
import math
from typing import Any, Dict, List, Optional, Set, Tuple

from rover.config import ArenaConfig, CameraConfig, PlannerConfig
from rover.models import CellStatus, CoverageStats

GridCoord = Tuple[int, int]


class ArenaGrid:
    def __init__(
        self,
        arena_cfg: Optional[ArenaConfig] = None,
        planner_cfg: Optional[PlannerConfig] = None,
        camera_cfg: Optional[CameraConfig] = None,
    ) -> None:
        self.arena_cfg = arena_cfg or ArenaConfig()
        self.planner_cfg = planner_cfg or PlannerConfig()
        self.camera_cfg = camera_cfg or CameraConfig()

        self.cols: int = int(math.floor((self.arena_cfg.width_m + 1e-9) / self.arena_cfg.cell_size_m))
        self.rows: int = int(math.floor((self.arena_cfg.height_m + 1e-9) / self.arena_cfg.cell_size_m))
        if self.cols < 1 or self.rows < 1:
            raise ValueError("Grid must have at least 1 column and 1 row.")

        self.remainder_x_m: float = max(0.0, round(self.arena_cfg.width_m - self.cols * self.arena_cfg.cell_size_m, 4))
        self.remainder_y_m: float = max(0.0, round(self.arena_cfg.height_m - self.rows * self.arena_cfg.cell_size_m, 4))
        self.wall_offset_x_m: float = self.remainder_x_m / 2.0
        self.wall_offset_y_m: float = self.remainder_y_m / 2.0

        self.clamp_margin_x_m: float = max(0.12, self.wall_offset_x_m + 0.5 * self.arena_cfg.cell_size_m)
        self.clamp_margin_y_m: float = max(0.12, self.wall_offset_y_m + 0.5 * self.arena_cfg.cell_size_m)

        self.configured_blocked: Set[GridCoord] = set()
        self.discovered_blocked: Set[GridCoord] = set()
        self.status_grid: Dict[GridCoord, CellStatus] = {}
        self.observed_grid: Dict[GridCoord, bool] = {}
        self.visit_counts: Dict[GridCoord, int] = {}
        self.log_odds_grid: Dict[GridCoord, float] = {}

        self.reset()

    def reset(self) -> None:
        self.configured_blocked.clear()
        self.discovered_blocked.clear()
        for c in range(self.cols):
            for r in range(self.rows):
                self.status_grid[(c, r)] = CellStatus.UNKNOWN
                self.observed_grid[(c, r)] = False
                self.visit_counts[(c, r)] = 0
                self.log_odds_grid[(c, r)] = 0.0

        for obs_c, obs_r in self.arena_cfg.configured_obstacles:
            if self.in_bounds(obs_c, obs_r):
                if (obs_c, obs_r) == (self.arena_cfg.home_col, self.arena_cfg.home_row):
                    raise ValueError("Home cell cannot be configured as an obstacle.")
                self.configured_blocked.add((obs_c, obs_r))
                self.status_grid[(obs_c, obs_r)] = CellStatus.BLOCKED
                self.log_odds_grid[(obs_c, obs_r)] = 3.5

        self.serpentine_order: List[GridCoord] = self._build_serpentine_order()
        self.scan_waypoints: Set[GridCoord] = self._build_scan_waypoints()
        self.update_reachability((self.arena_cfg.home_col, self.arena_cfg.home_row))

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_blocked(self, col: int, row: int) -> bool:
        if not self.in_bounds(col, row):
            return True
        return (
            (col, row) in self.configured_blocked
            or (col, row) in self.discovered_blocked
            or self.status_grid.get((col, row)) in (CellStatus.BLOCKED, CellStatus.HAZARD)
        )

    def is_traversable(self, col: int, row: int) -> bool:
        return self.in_bounds(col, row) and not self.is_blocked(col, row)

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        x = self.wall_offset_x_m + (col + 0.5) * self.arena_cfg.cell_size_m
        y = self.wall_offset_y_m + (row + 0.5) * self.arena_cfg.cell_size_m
        return (round(x, 4), round(y, 4))

    def world_to_cell(self, x_m: float, y_m: float) -> GridCoord:
        rel_x = x_m - self.wall_offset_x_m
        rel_y = y_m - self.wall_offset_y_m
        c = int(math.floor(rel_x / self.arena_cfg.cell_size_m))
        r = int(math.floor(rel_y / self.arena_cfg.cell_size_m))
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return (c, r)

    def _build_serpentine_order(self) -> List[GridCoord]:
        stride = max(1, int(self.planner_cfg.lane_stride))
        selected_rows = list(range(0, self.rows, stride))
        if (self.rows - 1) not in selected_rows:
            selected_rows.append(self.rows - 1)
        self.primary_lane_rows: Set[int] = set(selected_rows)

        order: List[GridCoord] = []
        seen: Set[GridCoord] = set()
        for lane_idx, r in enumerate(selected_rows):
            cols_iter = range(self.cols) if (lane_idx % 2 == 0) else range(self.cols - 1, -1, -1)
            for c in cols_iter:
                if (c, r) not in self.configured_blocked:
                    order.append((c, r))
                    seen.add((c, r))

        for r in range(self.rows):
            for c in range(self.cols):
                if (c, r) not in seen and (c, r) not in self.configured_blocked:
                    order.append((c, r))
                    seen.add((c, r))
        return order

    def _build_scan_waypoints(self) -> Set[GridCoord]:
        spacing = max(1, int(self.planner_cfg.scan_spacing))
        waypoints: Set[GridCoord] = set()
        for r in range(self.rows):
            lane_cells = [c for c in range(self.cols) if (c, r) not in self.configured_blocked]
            if not lane_cells:
                continue
            waypoints.add((lane_cells[0], r))
            waypoints.add((lane_cells[-1], r))
            for idx, c in enumerate(lane_cells):
                if idx % spacing == 0:
                    waypoints.add((c, r))
        return waypoints

    def should_scan_at(self, col: int, row: int) -> bool:
        return (col, row) in self.scan_waypoints

    def mark_visited(self, col: int, row: int, visited_observed: bool = True) -> None:
        if not self.in_bounds(col, row) or self.is_blocked(col, row):
            return
        self.status_grid[(col, row)] = CellStatus.VISITED
        self.visit_counts[(col, row)] = self.visit_counts.get((col, row), 0) + 1
        self.log_odds_grid[(col, row)] = max(-5.0, self.log_odds_grid.get((col, row), 0.0) - 2.4)
        if visited_observed:
            self.observed_grid[(col, row)] = True
        if getattr(self.planner_cfg, "fov_assist_visit", False) and self.planner_cfg.lane_stride >= 2:
            primary_rows = getattr(self, "primary_lane_rows", set(range(self.rows)))
            for dr in (-1, 1):
                nr = row + dr
                if 0 <= nr < self.rows and nr not in primary_rows and not self.is_blocked(col, nr):
                    self.status_grid[(col, nr)] = CellStatus.VISITED
                    self.observed_grid[(col, nr)] = True
                    self.log_odds_grid[(col, nr)] = max(-5.0, self.log_odds_grid.get((col, nr), 0.0) - 2.0)

    def mark_free(self, col: int, row: int) -> None:
        if not self.in_bounds(col, row) or self.is_blocked(col, row):
            return
        self.log_odds_grid[(col, row)] = max(-5.0, self.log_odds_grid.get((col, row), 0.0) - 1.386)
        if self.status_grid[(col, row)] in (CellStatus.UNKNOWN, CellStatus.UNREACHABLE):
            self.status_grid[(col, row)] = CellStatus.FREE

    def mark_uncertain(self, col: int, row: int) -> None:
        if not self.in_bounds(col, row) or self.is_blocked(col, row):
            return
        if self.status_grid[(col, row)] != CellStatus.VISITED:
            self.status_grid[(col, row)] = CellStatus.UNCERTAIN

    def mark_discovered_obstacle(self, col: int, row: int, current_cell: Optional[GridCoord] = None) -> bool:
        if not self.in_bounds(col, row):
            return False
        if current_cell is not None and (col, row) == current_cell:
            return False
        if (col, row) == (self.arena_cfg.home_col, self.arena_cfg.home_row):
            return False
        self.discovered_blocked.add((col, row))
        self.status_grid[(col, row)] = CellStatus.BLOCKED
        self.log_odds_grid[(col, row)] = min(5.0, self.log_odds_grid.get((col, row), 0.0) + 2.8)
        origin = current_cell or (self.arena_cfg.home_col, self.arena_cfg.home_row)
        self.update_reachability(origin)
        return True

    def cell_ahead(self, col: int, row: int, heading_deg: float) -> GridCoord:
        rad = math.radians(heading_deg)
        if abs(math.cos(rad)) >= abs(math.sin(rad)):
            dc = 1 if math.cos(rad) >= 0 else -1
            dr = 0
        else:
            dc = 0
            dr = 1 if math.sin(rad) >= 0 else -1
        return (col + dc, row + dr)

    def mark_discovered_hazard(self, col: int, row: int, current_cell: Optional[GridCoord] = None) -> bool:
        if not self.in_bounds(col, row):
            return False
        if current_cell is not None and (col, row) == current_cell:
            return False
        if (col, row) == (self.arena_cfg.home_col, self.arena_cfg.home_row):
            return False
        self.discovered_blocked.add((col, row))
        self.status_grid[(col, row)] = CellStatus.HAZARD
        self.log_odds_grid[(col, row)] = min(5.0, self.log_odds_grid.get((col, row), 0.0) + 3.0)
        origin = current_cell or (self.arena_cfg.home_col, self.arena_cfg.home_row)
        self.update_reachability(origin)
        return True

    def set_configured_obstacle(self, col: int, row: int, blocked: bool) -> None:
        if not self.in_bounds(col, row):
            return
        if blocked and (col, row) == (self.arena_cfg.home_col, self.arena_cfg.home_row):
            raise ValueError("Cannot place obstacle on Home cell.")
        if blocked:
            self.configured_blocked.add((col, row))
            self.status_grid[(col, row)] = CellStatus.BLOCKED
        else:
            self.configured_blocked.discard((col, row))
            self.discovered_blocked.discard((col, row))
            self.status_grid[(col, row)] = CellStatus.UNKNOWN
        self.serpentine_order = self._build_serpentine_order()
        self.scan_waypoints = self._build_scan_waypoints()
        self.update_reachability((self.arena_cfg.home_col, self.arena_cfg.home_row))

    def neighbors4(self, col: int, row: int) -> List[GridCoord]:
        out: List[GridCoord] = []
        for dc, dr in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            nc, nr = col + dc, row + dr
            if self.is_traversable(nc, nr):
                out.append((nc, nr))
        return out

    def update_reachability(self, start: GridCoord) -> Set[GridCoord]:
        reachable: Set[GridCoord] = set()
        if self.in_bounds(*start) and not self.is_blocked(*start):
            queue = [start]
            reachable.add(start)
            while queue:
                curr = queue.pop(0)
                for nb in self.neighbors4(*curr):
                    if nb not in reachable:
                        reachable.add(nb)
                        queue.append(nb)

        for c in range(self.cols):
            for r in range(self.rows):
                coord = (c, r)
                if self.is_blocked(c, r):
                    if self.status_grid.get(coord) != CellStatus.HAZARD:
                        self.status_grid[coord] = CellStatus.BLOCKED
                elif coord not in reachable:
                    if self.status_grid[coord] != CellStatus.VISITED:
                        self.status_grid[coord] = CellStatus.UNREACHABLE
                else:
                    if self.status_grid[coord] == CellStatus.UNREACHABLE:
                        self.status_grid[coord] = CellStatus.UNKNOWN
        return reachable

    def astar_path(self, start: GridCoord, goal: GridCoord) -> Optional[List[GridCoord]]:
        if not self.in_bounds(*start) or not self.is_traversable(*goal):
            return None
        if start == goal:
            return [start]

        def heuristic(a: GridCoord, b: GridCoord) -> float:
            return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))

        open_heap: List[Tuple[float, int, GridCoord]] = []
        counter = 0
        heapq.heappush(open_heap, (heuristic(start, goal), counter, start))
        came_from: Dict[GridCoord, GridCoord] = {}
        g_score: Dict[GridCoord, float] = {start: 0.0}

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nb in self.neighbors4(*current):
                turn_penalty = 0.0
                if current in came_from:
                    prev = came_from[current]
                    if (current[0] - prev[0], current[1] - prev[1]) != (nb[0] - current[0], nb[1] - current[1]):
                        turn_penalty = 0.05
                tentative_g = g_score[current] + 1.0 + turn_penalty
                if tentative_g < g_score.get(nb, float("inf")):
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    counter += 1
                    f_score = tentative_g + heuristic(nb, goal)
                    heapq.heappush(open_heap, (f_score, counter, nb))
        return None

    def plan_rescue_route_from_home(
        self,
        target_col: int,
        target_row: int,
        target_x_m: Optional[float] = None,
        target_y_m: Optional[float] = None,
    ) -> Tuple[List[List[int]], float, float, str]:
        home = (self.arena_cfg.home_col, self.arena_cfg.home_row)
        hx, hy = self.cell_to_world(*home)
        if target_x_m is None or target_y_m is None:
            tx, ty = self.cell_to_world(target_col, target_row)
        else:
            tx, ty = target_x_m, target_y_m
        straight_dist_m = round(math.hypot(tx - hx, ty - hy), 2)

        goal = (target_col, target_row)
        path = self.astar_path(home, goal) if self.is_traversable(*goal) else None
        if path is None:
            candidates: List[Tuple[int, List[GridCoord]]] = []
            for nb in self.neighbors4(target_col, target_row):
                p = self.astar_path(home, nb)
                if p is not None:
                    candidates.append((len(p), p))
            if candidates:
                candidates.sort(key=lambda item: item[0])
                path = candidates[0][1]

        if not path:
            return ([], straight_dist_m, 0.0, f"Home ({home[0]},{home[1]}) -> No clear path to ({target_col},{target_row})")

        path_list = [[c, r] for c, r in path]
        path_dist_m = round(max(0, len(path) - 1) * self.arena_cfg.cell_size_m, 2)
        directions = self.describe_path_segments(path)
        return (path_list, straight_dist_m, path_dist_m, directions)

    def describe_path_segments(self, path: List[GridCoord]) -> str:
        if not path:
            return "No route"
        if len(path) == 1:
            return f"At Home ({path[0][0]},{path[0][1]})"

        segments: List[str] = [f"Home ({path[0][0]},{path[0][1]})"]
        i = 0
        while i < len(path) - 1:
            dc = path[i + 1][0] - path[i][0]
            dr = path[i + 1][1] - path[i][1]
            steps = 1
            while (
                i + steps < len(path) - 1
                and (path[i + steps + 1][0] - path[i + steps][0]) == dc
                and (path[i + steps + 1][1] - path[i + steps][1]) == dr
            ):
                steps += 1
            if dc > 0:
                card = "East"
            elif dc < 0:
                card = "West"
            elif dr > 0:
                card = "North"
            else:
                card = "South"
            dist_m = steps * self.arena_cfg.cell_size_m
            segments.append(f"{card} {dist_m:.2f}m ({steps} cell{'s' if steps > 1 else ''})")
            i += steps

        end_c, end_r = path[-1]
        segments.append(f"Reach ({end_c},{end_r})")
        return " → ".join(segments)

    def next_coverage_target(
        self,
        current: GridCoord,
        pending_ai_confirm_cell: Optional[GridCoord] = None,
    ) -> Tuple[Optional[GridCoord], Optional[List[GridCoord]], bool]:
        self.update_reachability(current)

        if (
            self.planner_cfg.smart_confirm_enabled
            and pending_ai_confirm_cell is not None
            and self.is_traversable(*pending_ai_confirm_cell)
        ):
            path = self.astar_path(current, pending_ai_confirm_cell)
            if path is not None:
                return pending_ai_confirm_cell, path, True

        unvisited: List[GridCoord] = [
            coord
            for coord in self.serpentine_order
            if not self.is_blocked(*coord)
            and self.status_grid.get(coord) in (CellStatus.UNKNOWN, CellStatus.FREE, CellStatus.UNCERTAIN)
        ]
        if not unvisited:
            return None, None, False

        if self.planner_cfg.coverage_mode == "serpentine":
            for candidate in unvisited:
                path = self.astar_path(current, candidate)
                if path is not None:
                    return candidate, path, False
            return None, None, False

        if self.planner_cfg.coverage_mode == "frontier_ig":
            best_ig_target = None
            best_ig_path = None
            best_utility = -1e9
            for rank, candidate in enumerate(unvisited):
                path = self.astar_path(current, candidate)
                if path is None:
                    continue
                ig_score = self.estimate_information_gain(candidate[0], candidate[1])
                travel_cost = float(max(1, len(path) - 1))
                utility = (2.2 * ig_score) - (1.0 * travel_cost) - (0.001 * rank)
                if utility > best_utility:
                    best_utility = utility
                    best_ig_target = candidate
                    best_ig_path = path
            return best_ig_target, best_ig_path, False

        best_target = None
        best_path = None
        best_key = (10**9, 10**9)
        for rank, candidate in enumerate(unvisited):
            path = self.astar_path(current, candidate)
            if path is not None:
                key = (len(path), rank)
                if key < best_key:
                    best_key = key
                    best_target = candidate
                    best_path = path
        return best_target, best_path, False

    def compute_ai_confirmation_step(
        self,
        rover_cell: GridCoord,
        rover_heading_deg: float,
        relative_bearing_deg: float,
    ) -> GridCoord:
        abs_bearing_rad = math.radians(rover_heading_deg + relative_bearing_deg)
        dc = int(round(math.cos(abs_bearing_rad)))
        dr = int(round(math.sin(abs_bearing_rad)))
        if abs(math.cos(abs_bearing_rad)) >= abs(math.sin(abs_bearing_rad)):
            step_cell = (rover_cell[0] + (1 if math.cos(abs_bearing_rad) >= 0 else -1), rover_cell[1])
        else:
            step_cell = (rover_cell[0], rover_cell[1] + (1 if math.sin(abs_bearing_rad) >= 0 else -1))

        if self.is_traversable(*step_cell):
            return step_cell
        alt_cell = (rover_cell[0] + dc, rover_cell[1]) if dr != 0 else (rover_cell[0], rover_cell[1] + dr)
        if alt_cell != rover_cell and self.is_traversable(*alt_cell):
            return alt_cell
        return rover_cell

    def raycast_observed_fov(
        self,
        rover_x_m: float,
        rover_y_m: float,
        rover_heading_deg: float,
        hfov_deg: Optional[float] = None,
        max_range_m: Optional[float] = None,
    ) -> List[GridCoord]:
        fov = hfov_deg if hfov_deg is not None else self.camera_cfg.hfov_deg
        rng = max_range_m if max_range_m is not None else self.camera_cfg.detect_range_m

        newly_observed: Set[GridCoord] = set()
        rover_cell = self.world_to_cell(rover_x_m, rover_y_m)
        if self.is_traversable(*rover_cell):
            self.observed_grid[rover_cell] = True
            newly_observed.add(rover_cell)

        num_rays = 31
        step_m = min(0.05, self.arena_cfg.cell_size_m * 0.25)
        half_fov = 0.5 * fov

        for i in range(num_rays):
            angle_offset = -half_fov + (fov * i / max(1, num_rays - 1))
            ray_rad = math.radians(rover_heading_deg + angle_offset)
            cos_a = math.cos(ray_rad)
            sin_a = math.sin(ray_rad)

            dist = step_m
            while dist <= rng + 1e-6:
                px = rover_x_m + dist * cos_a
                py = rover_y_m + dist * sin_a

                if (
                    px < self.wall_offset_x_m
                    or px > self.arena_cfg.width_m - self.wall_offset_x_m
                    or py < self.wall_offset_y_m
                    or py > self.arena_cfg.height_m - self.wall_offset_y_m
                ):
                    break

                c, r = self.world_to_cell(px, py)
                if not self.in_bounds(c, r) or self.is_blocked(c, r):
                    break

                self.observed_grid[(c, r)] = True
                self.mark_free(c, r)
                newly_observed.add((c, r))
                dist += step_m

        return sorted(newly_observed)

    def cell_occupancy_prob(self, col: int, row: int) -> float:
        l_val = self.log_odds_grid.get((col, row), 0.0)
        return 1.0 / (1.0 + math.exp(-l_val))

    def cell_shannon_entropy(self, col: int, row: int) -> float:
        p = max(1e-6, min(1.0 - 1e-6, self.cell_occupancy_prob(col, row)))
        return float(-p * math.log2(p) - (1.0 - p) * math.log2(1.0 - p))

    def is_frontier_cell(self, col: int, row: int) -> bool:
        if not self.is_traversable(col, row):
            return False
        st = self.status_grid.get((col, row), CellStatus.UNKNOWN)
        if st not in (CellStatus.FREE, CellStatus.VISITED):
            return False
        for nb in self.neighbors4(col, row):
            if self.status_grid.get(nb, CellStatus.UNKNOWN) == CellStatus.UNKNOWN:
                return True
        return False

    def estimate_information_gain(self, col: int, row: int) -> float:
        gain = 0.0
        for dc in range(-2, 3):
            for dr in range(-2, 3):
                nc, nr = col + dc, row + dr
                if self.in_bounds(nc, nr) and not self.is_blocked(nc, nr):
                    if not self.observed_grid.get((nc, nr), False):
                        gain += self.cell_shannon_entropy(nc, nr)
                    elif self.status_grid.get((nc, nr)) != CellStatus.VISITED:
                        gain += 0.35 * self.cell_shannon_entropy(nc, nr)
        return round(gain, 3)

    def compute_coverage_stats(self) -> CoverageStats:
        total_cells = self.cols * self.rows
        cfg_blocked = len(self.configured_blocked)
        disc_blocked = len(self.discovered_blocked - self.configured_blocked)

        eligible = 0
        visited = 0
        observed = 0
        unknown = 0
        free = 0
        uncertain = 0
        unreachable = 0
        total_entropy_bits = 0.0
        frontier_count = 0

        for c in range(self.cols):
            for r in range(self.rows):
                coord = (c, r)
                st = self.status_grid.get(coord, CellStatus.UNKNOWN)
                if self.is_blocked(c, r):
                    continue
                eligible += 1
                total_entropy_bits += self.cell_shannon_entropy(c, r)
                if self.is_frontier_cell(c, r):
                    frontier_count += 1

                if st == CellStatus.VISITED:
                    visited += 1
                elif st == CellStatus.UNKNOWN:
                    unknown += 1
                elif st == CellStatus.FREE:
                    free += 1
                elif st == CellStatus.UNCERTAIN:
                    uncertain += 1
                elif st == CellStatus.UNREACHABLE:
                    unreachable += 1

                if self.observed_grid.get(coord, False):
                    observed += 1

        if eligible > 0:
            visited_pct = (visited / eligible) * 100.0
            observed_pct = (observed / eligible) * 100.0
            if visited < eligible and visited_pct >= 99.95:
                visited_pct = 99.9
            if observed < eligible and observed_pct >= 99.95:
                observed_pct = 99.9
            max_entropy = float(eligible)
            entropy_red_pct = max(0.0, min(100.0, ((max_entropy - total_entropy_bits) / max_entropy) * 100.0))
        else:
            visited_pct = 0.0
            observed_pct = 0.0
            entropy_red_pct = 0.0

        return CoverageStats(
            cols=self.cols,
            rows=self.rows,
            total_cells=total_cells,
            configured_blocked_cells=cfg_blocked,
            discovered_blocked_cells=disc_blocked,
            eligible_cells=eligible,
            visited_cells=visited,
            observed_cells=observed,
            unknown_cells=unknown,
            free_cells=free,
            uncertain_cells=uncertain,
            unreachable_cells=unreachable,
            visited_coverage_pct=round(visited_pct, 2),
            observed_coverage_pct=round(observed_pct, 2),
            margin_x_m=self.remainder_x_m,
            margin_y_m=self.remainder_y_m,
            shannon_entropy_bits=round(total_entropy_bits, 2),
            entropy_reduction_pct=round(entropy_red_pct, 1),
            frontier_cells_count=frontier_count,
        )

    def to_dict(self) -> Dict[str, Any]:
        cells = []
        for r in range(self.rows):
            row_list = []
            for c in range(self.cols):
                coord = (c, r)
                row_list.append(
                    {
                        "col": c,
                        "row": r,
                        "status": self.status_grid[coord].value,
                        "observed": bool(self.observed_grid.get(coord, False)),
                        "configured_blocked": coord in self.configured_blocked,
                        "discovered_blocked": coord in self.discovered_blocked,
                        "scan_waypoint": coord in self.scan_waypoints,
                        "visits": self.visit_counts.get(coord, 0),
                        "log_odds": round(self.log_odds_grid.get(coord, 0.0), 2),
                        "entropy": round(self.cell_shannon_entropy(c, r), 2),
                        "frontier": self.is_frontier_cell(c, r),
                    }
                )
            cells.append(row_list)
        return {
            "width_m": self.arena_cfg.width_m,
            "height_m": self.arena_cfg.height_m,
            "cell_size_m": self.arena_cfg.cell_size_m,
            "cols": self.cols,
            "rows": self.rows,
            "margin_x_m": self.remainder_x_m,
            "margin_y_m": self.remainder_y_m,
            "home": [self.arena_cfg.home_col, self.arena_cfg.home_row],
            "configured_obstacles": [list(x) for x in sorted(self.configured_blocked)],
            "discovered_obstacles": [list(x) for x in sorted(self.discovered_blocked)],
            "cells": cells,
            "stats": self.compute_coverage_stats().to_dict(),
        }
