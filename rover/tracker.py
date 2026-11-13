"""Multi-frame burst tracking, 17-keypoint pose triage, distress gesture detection, and target coordinate projection."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from rover.config import CameraConfig, VisionConfig
from rover.grid import ArenaGrid
from rover.models import DetectionBox, PoseState, SightingCandidate


def compute_iou(a: DetectionBox, b: DetectionBox) -> float:
    ix1 = max(a.x1, b.x1)
    iy1 = max(a.y1, b.y1)
    ix2 = min(a.x2, b.x2)
    iy2 = min(a.y2, b.y2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = a.width_px * a.height_px + b.width_px * b.height_px - inter
    return inter / max(1e-6, union)


def analyze_pose_triage(box: DetectionBox) -> Tuple[str, str, bool, float, float]:
    """
    Analyze 17-keypoint COCO pose skeleton (if available) to detect:
    - Distress waving gesture (wrist above shoulder/nose) -> P1-CRITICAL (WAVING FOR HELP)
    - Spine/Torso inclination angle (shoulder midpoint -> hip midpoint)
    Falls back seamlessly to bounding-box aspect ratio if keypoints are absent.
    Returns (posture_label, triage_priority, is_waving, torso_angle_deg, aspect_ratio).
    """
    ar = box.width_px / max(1.0, box.height_px)
    kpts = box.keypoints
    is_waving = False
    torso_angle = 0.0

    if kpts and len(kpts) >= 13:
        # COCO Keypoints: 0=nose, 5=L_shoulder, 6=R_shoulder, 9=L_wrist, 10=R_wrist, 11=L_hip, 12=R_hip
        shoulders = [kpts[i] for i in (5, 6) if kpts[i][2] > 0.30]
        hips = [kpts[i] for i in (11, 12) if kpts[i][2] > 0.30]
        wrists = [kpts[i] for i in (9, 10) if kpts[i][2] > 0.30]

        if shoulders and wrists:
            avg_sh_y = sum(s[1] for s in shoulders) / len(shoulders)
            for w in wrists:
                if w[1] < avg_sh_y - 10.0:
                    is_waving = True
                    break

        if shoulders and hips:
            sx = sum(s[0] for s in shoulders) / len(shoulders)
            sy = sum(s[1] for s in shoulders) / len(shoulders)
            hx = sum(h[0] for h in hips) / len(hips)
            hy = sum(h[1] for h in hips) / len(hips)
            dx = abs(sx - hx)
            dy = abs(hy - sy)
            torso_angle = math.degrees(math.atan2(dx, max(1.0, dy)))

            if is_waving:
                return ("WAVING FOR HELP", "P1-CRITICAL", True, round(torso_angle, 1), round(ar, 2))
            if torso_angle >= 52.0 or ar >= 1.05:
                return ("PRONE / FALLEN", "P1-CRITICAL", False, round(torso_angle, 1), round(ar, 2))
            if torso_angle >= 28.0 or ar >= 0.68:
                return ("SEATED / CROUCHED", "P2-URGENT", False, round(torso_angle, 1), round(ar, 2))
            return ("STANDING", "P3-AMBULATORY", False, round(torso_angle, 1), round(ar, 2))

        if is_waving:
            return ("WAVING FOR HELP", "P1-CRITICAL", True, 0.0, round(ar, 2))

    # Fallback: aspect-ratio geometric triage
    if ar >= 1.05:
        return ("PRONE / FALLEN", "P1-CRITICAL", False, 75.0, round(ar, 2))
    if ar >= 0.68:
        return ("SEATED / CROUCHED", "P2-URGENT", False, 35.0, round(ar, 2))
    return ("STANDING", "P3-AMBULATORY", False, 5.0, round(ar, 2))


class MultiFrameBurstTracker:
    def __init__(
        self,
        grid: ArenaGrid,
        vision_cfg: Optional[VisionConfig] = None,
        camera_cfg: Optional[CameraConfig] = None,
    ) -> None:
        self.grid = grid
        self.vision_cfg = vision_cfg or VisionConfig()
        self.camera_cfg = camera_cfg or CameraConfig()

    def project_box_to_arena(
        self,
        box: DetectionBox,
        rover_pose: PoseState,
    ) -> Tuple[float, float, float, float, int, int]:
        h_px = max(1.0, box.height_px)
        raw_dist = (self.vision_cfg.focal_scale_px * self.vision_cfg.assumed_person_height_m) / h_px
        est_dist_m = max(0.5, min(3.0, raw_dist))

        norm_x = (0.5 * self.camera_cfg.width - box.center_x_px) / max(1.0, 0.5 * self.camera_cfg.width)
        bearing_deg = norm_x * (0.5 * self.camera_cfg.hfov_deg)

        abs_rad = math.radians(rover_pose.heading_deg + bearing_deg)
        raw_x = rover_pose.x_m + est_dist_m * math.cos(abs_rad)
        raw_y = rover_pose.y_m + est_dist_m * math.sin(abs_rad)

        clamped_x = max(0.05, min(self.grid.arena_cfg.width_m - 0.05, raw_x))
        clamped_y = max(0.05, min(self.grid.arena_cfg.height_m - 0.05, raw_y))
        c, r = self.grid.world_to_cell(clamped_x, clamped_y)
        return (round(bearing_deg, 2), round(est_dist_m, 2), round(clamped_x, 3), round(clamped_y, 3), c, r)

    def evaluate_burst(
        self,
        burst_detections: List[List[DetectionBox]],
        rover_pose: PoseState,
    ) -> List[SightingCandidate]:
        tracks: List[List[DetectionBox]] = []

        for frame_idx, frame_boxes in enumerate(burst_detections):
            used_track_indices = set()
            for box in frame_boxes:
                if (
                    box.confidence < self.vision_cfg.tentative_conf_min
                    or box.height_px < self.vision_cfg.tentative_min_box_height_px
                ):
                    continue
                best_track_idx: Optional[int] = None
                best_iou = self.vision_cfg.iou_match_threshold
                for t_idx, track in enumerate(tracks):
                    if t_idx in used_track_indices:
                        continue
                    if any(b.frame_index == box.frame_index for b in track):
                        continue
                    iou = compute_iou(track[-1], box)
                    if iou >= best_iou:
                        best_iou = iou
                        best_track_idx = t_idx

                if best_track_idx is not None:
                    tracks[best_track_idx].append(box)
                    used_track_indices.add(best_track_idx)
                else:
                    tracks.append([box])
                    used_track_indices.add(len(tracks) - 1)

        candidates: List[SightingCandidate] = []
        for track in tracks:
            frames_seen = len(track)
            mean_conf = sum(b.confidence for b in track) / frames_seen
            max_h = max(b.height_px for b in track)
            rep_box = max(track, key=lambda b: b.confidence)

            is_confirmed = (
                frames_seen >= self.vision_cfg.min_confirm_frames
                and mean_conf >= self.vision_cfg.confirm_conf_min
                and max_h >= self.vision_cfg.min_box_height_px
            )
            is_tentative = (not is_confirmed) and (
                mean_conf >= self.vision_cfg.tentative_conf_min
                and max_h >= self.vision_cfg.tentative_min_box_height_px
            )
            if not is_confirmed and not is_tentative:
                continue

            bearing, dist_m, tx, ty, tc, tr = self.project_box_to_arena(rep_box, rover_pose)
            posture_label, triage_priority, is_waving, torso_angle, ar = analyze_pose_triage(rep_box)
            ibvs_turn = round(0.85 * bearing, 2)

            candidates.append(
                SightingCandidate(
                    boxes=track,
                    mean_confidence=mean_conf,
                    max_height_px=max_h,
                    frames_seen=frames_seen,
                    confirmed=is_confirmed,
                    tentative=is_tentative,
                    bearing_deg=bearing,
                    est_distance_m=dist_m,
                    est_x_m=tx,
                    est_y_m=ty,
                    est_col=tc,
                    est_row=tr,
                    posture_label=posture_label,
                    triage_priority=triage_priority,
                    aspect_ratio=ar,
                    ibvs_turn_deg=ibvs_turn,
                    is_waving=is_waving,
                    torso_angle_deg=torso_angle,
                )
            )
        return candidates
