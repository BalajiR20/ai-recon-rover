"""Unit tests for multi-frame confirmation, tentative sightings, and spatial cross-visit target association."""
from __future__ import annotations

import pytest
from rover.association import TargetAssociator
from rover.config import ArenaConfig, AssociationConfig, CameraConfig, VisionConfig
from rover.grid import ArenaGrid
from rover.models import AssociationLabel, DetectionBox, PoseState
from rover.tracker import MultiFrameBurstTracker


def test_multiframe_confirmation_and_tentative_flagging() -> None:
    grid = ArenaGrid(ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30))
    tracker = MultiFrameBurstTracker(grid, VisionConfig(), CameraConfig())
    pose = PoseState(x_m=0.45, y_m=0.55, heading_deg=0.0)

    # 1. Single-frame detection with high conf -> NOT confirmed (seen in 1 < 2 frames), flagged tentative
    burst_single = [
        [DetectionBox(x1=280, y1=180, x2=360, y2=300, confidence=0.65, frame_index=0)],
        [],
        [],
    ]
    cands_single = tracker.evaluate_burst(burst_single, pose)
    assert len(cands_single) == 1
    assert cands_single[0].confirmed is False
    assert cands_single[0].tentative is True

    # 2. Seen in 2 of 3 frames with high confidence and height >= 36 px -> CONFIRMED
    burst_confirmed = [
        [DetectionBox(x1=280, y1=180, x2=360, y2=300, confidence=0.72, frame_index=0)],
        [DetectionBox(x1=282, y1=181, x2=362, y2=301, confidence=0.75, frame_index=1)],
        [],
    ]
    cands_conf = tracker.evaluate_burst(burst_confirmed, pose)
    assert len(cands_conf) == 1
    assert cands_conf[0].confirmed is True
    assert cands_conf[0].tentative is False

    # 3. Borderline confidence (0.26) in 2 frames -> TENTATIVE (triggers Smart AI Confirmation)
    burst_borderline = [
        [DetectionBox(x1=280, y1=180, x2=360, y2=300, confidence=0.26, frame_index=0)],
        [DetectionBox(x1=281, y1=180, x2=361, y2=300, confidence=0.28, frame_index=1)],
        [],
    ]
    cands_tent = tracker.evaluate_burst(burst_borderline, pose)
    assert len(cands_tent) == 1
    assert cands_tent[0].confirmed is False
    assert cands_tent[0].tentative is True


def test_cross_visit_spatial_association_and_simultaneous_non_merge() -> None:
    grid = ArenaGrid(ArenaConfig(width_m=3.0, height_m=2.0, cell_size_m=0.30))
    tracker = MultiFrameBurstTracker(grid, VisionConfig(), CameraConfig())
    assoc = TargetAssociator(grid, AssociationConfig())
    pose = PoseState(x_m=0.45, y_m=0.55, heading_deg=0.0, sigma_pos_m=0.08)

    # First scan: two distinct people visible simultaneously in the same frame (left and right)
    left_p0 = DetectionBox(x1=80, y1=150, x2=160, y2=310, confidence=0.80, frame_index=0)
    right_p0 = DetectionBox(x1=480, y1=150, x2=560, y2=310, confidence=0.82, frame_index=0)
    left_p1 = DetectionBox(x1=82, y1=151, x2=162, y2=311, confidence=0.79, frame_index=1)
    right_p1 = DetectionBox(x1=478, y1=149, x2=558, y2=309, confidence=0.84, frame_index=1)

    cands = tracker.evaluate_burst([[left_p0, right_p0], [left_p1, right_p1], []], pose)
    confirmed = [c for c in cands if c.confirmed]
    assert len(confirmed) == 2

    records = assoc.associate_burst_candidates(confirmed, pose)
    # Must never merge two distinct people seen simultaneously in the same frame!
    assert len(records) == 2
    assert len(assoc.targets) == 2
    assert records[0].target_id == "T01"
    assert records[1].target_id == "T02"

    # Second scan slightly later seeing the left person within 0.60 m -> LIKELY_SAME_TARGET
    cands_revisit = tracker.evaluate_burst([[left_p0], [left_p1], []], pose)
    recs_revisit = assoc.associate_burst_candidates([c for c in cands_revisit if c.confirmed], pose)
    assert len(recs_revisit) == 1
    assert recs_revisit[0].target_id == "T01"
    assert recs_revisit[0].association_label == AssociationLabel.LIKELY_SAME_TARGET.value
    assert recs_revisit[0].sightings_count == 2
