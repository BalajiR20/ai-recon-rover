"""Spatial target association, triage tracking, and multi-target dispatch tour."""
from __future__ import annotations

import math
import time
from typing import List, Optional, Set

from rover.config import AssociationConfig
from rover.grid import ArenaGrid
from rover.models import AssociationLabel, PoseState, SightingCandidate, TargetRecord


class TargetAssociator:
    def __init__(self, grid: ArenaGrid, assoc_cfg: Optional[AssociationConfig] = None) -> None:
        self.grid = grid
        self.assoc_cfg = assoc_cfg or AssociationConfig()
        self.targets: List[TargetRecord] = []

    def refresh_all_rescue_paths(self) -> None:
        for t in self.targets:
            path_from_home, straight_d, path_d, directions = self.grid.plan_rescue_route_from_home(
                t.col, t.row, t.x_m, t.y_m
            )
            t.path_from_home = path_from_home
            t.distance_from_home_m = straight_d
            t.rescue_path_distance_m = path_d
            t.rescue_directions = directions

    def compute_multi_target_rescue_tour(self) -> dict:
        home = (self.grid.arena_cfg.home_col, self.grid.arena_cfg.home_row)
        if not self.targets:
            return {
                "ordered_target_ids": [],
                "total_tour_distance_m": 0.0,
                "tour_summary": f"Home ({home[0]},{home[1]}) — Standby (No casualties logged)",
                "tour_cells": [[home[0], home[1]]],
            }

        priority_rank = {"P1-CRITICAL": 1, "P2-URGENT": 2, "P3-AMBULATORY": 3}
        remaining = list(self.targets)
        curr_cell = home
        ordered_ids = []
        full_cells = [[home[0], home[1]]]
        total_steps = 0
        leg_descriptions = [f"Home ({home[0]},{home[1]})"]

        while remaining:
            best_t = None
            best_key = (999, 999999.0)
            best_path = None
            for t in remaining:
                p_rank = priority_rank.get(t.triage_priority, 3)
                goal = (t.col, t.row)
                path = self.grid.astar_path(curr_cell, goal) if self.grid.is_traversable(*goal) else None
                steps = (len(path) - 1) if path else int(round(math.hypot(t.col - curr_cell[0], t.row - curr_cell[1])))
                key = (p_rank, float(steps))
                if key < best_key:
                    best_key = key
                    best_t = t
                    best_path = path
            if best_t is None:
                break
            remaining.remove(best_t)
            ordered_ids.append(best_t.target_id)
            if best_path and len(best_path) > 1:
                for c, r in best_path[1:]:
                    full_cells.append([c, r])
                total_steps += len(best_path) - 1
            else:
                full_cells.append([best_t.col, best_t.row])
            curr_cell = (best_t.col, best_t.row)
            leg_descriptions.append(f"{best_t.target_id} [{best_t.triage_priority} at ({best_t.col},{best_t.row})]")

        total_dist_m = round(total_steps * self.grid.arena_cfg.cell_size_m, 2)
        return {
            "ordered_target_ids": ordered_ids,
            "total_tour_distance_m": total_dist_m,
            "tour_summary": " → ".join(leg_descriptions),
            "tour_cells": full_cells,
        }

    def associate_burst_candidates(
        self,
        confirmed_candidates: List[SightingCandidate],
        rover_pose: PoseState,
    ) -> List[TargetRecord]:
        updated_records: List[TargetRecord] = []
        matched_target_ids: Set[str] = set()

        now = time.time()
        for idx, cand in enumerate(confirmed_candidates):
            best_target: Optional[TargetRecord] = None
            best_dist = float("inf")
            gate_max = self.assoc_cfg.possible_match_dist_m + rover_pose.sigma_pos_m

            for existing in self.targets:
                if existing.target_id in matched_target_ids:
                    continue
                d = math.hypot(cand.est_x_m - existing.x_m, cand.est_y_m - existing.y_m)
                if d <= gate_max and d < best_dist:
                    best_dist = d
                    best_target = existing

            if best_target is not None and best_dist <= gate_max:
                matched_target_ids.add(best_target.target_id)
                label = AssociationLabel.LIKELY_SAME_TARGET.value if best_dist <= self.assoc_cfg.likely_same_dist_m else AssociationLabel.POSSIBLE_MATCH.value

                w = 1.0 / (best_target.sightings_count + 1)
                best_target.x_m = round((1.0 - w) * best_target.x_m + w * cand.est_x_m, 3)
                best_target.y_m = round((1.0 - w) * best_target.y_m + w * cand.est_y_m, 3)
                best_target.col, best_target.row = self.grid.world_to_cell(best_target.x_m, best_target.y_m)
                best_target.confidence = round(max(best_target.confidence, cand.mean_confidence), 3)
                best_target.frames_seen = max(best_target.frames_seen, cand.frames_seen)
                best_target.sightings_count += 1
                best_target.last_seen_ts = now
                best_target.association_label = label
                best_target.posture_label = cand.posture_label
                best_target.triage_priority = cand.triage_priority
                best_target.aspect_ratio = cand.aspect_ratio
                best_target.ibvs_turn_deg = cand.ibvs_turn_deg
                best_target.is_waving = cand.is_waving
                best_target.torso_angle_deg = cand.torso_angle_deg
                if cand.snapshot_path:
                    best_target.snapshot_path = cand.snapshot_path

                path_from_home, straight_d, path_d, directions = self.grid.plan_rescue_route_from_home(
                    best_target.col, best_target.row, best_target.x_m, best_target.y_m
                )
                best_target.path_from_home = path_from_home
                best_target.distance_from_home_m = straight_d
                best_target.rescue_path_distance_m = path_d
                best_target.rescue_directions = directions
                updated_records.append(best_target)
            else:
                new_id = f"T{len(self.targets) + 1:02d}"
                path_from_home, straight_d, path_d, directions = self.grid.plan_rescue_route_from_home(
                    cand.est_col, cand.est_row, cand.est_x_m, cand.est_y_m
                )
                rec = TargetRecord(
                    target_id=new_id,
                    association_label=AssociationLabel.NEW_TARGET.value,
                    x_m=cand.est_x_m,
                    y_m=cand.est_y_m,
                    col=cand.est_col,
                    row=cand.est_row,
                    confidence=round(cand.mean_confidence, 3),
                    frames_seen=cand.frames_seen,
                    sightings_count=1,
                    first_seen_ts=now,
                    last_seen_ts=now,
                    rover_x_m=rover_pose.x_m,
                    rover_y_m=rover_pose.y_m,
                    rover_heading_deg=rover_pose.heading_deg,
                    sigma_pos_m=rover_pose.sigma_pos_m,
                    snapshot_path=cand.snapshot_path,
                    distance_from_home_m=straight_d,
                    rescue_path_distance_m=path_d,
                    path_from_home=path_from_home,
                    rescue_directions=directions,
                    posture_label=cand.posture_label,
                    triage_priority=cand.triage_priority,
                    aspect_ratio=cand.aspect_ratio,
                    ibvs_turn_deg=cand.ibvs_turn_deg,
                    is_waving=cand.is_waving,
                    torso_angle_deg=cand.torso_angle_deg,
                )
                self.targets.append(rec)
                matched_target_ids.add(new_id)
                updated_records.append(rec)

        return updated_records
