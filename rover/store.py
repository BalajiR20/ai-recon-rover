"""Persistence store for events, snapshots, coverage map, and target database."""
from __future__ import annotations

import csv
import datetime
import io
import json
import pathlib
import threading
import time
from typing import Any, Dict, List, Optional
import cv2
import numpy as np

from rover.grid import ArenaGrid
from rover.models import DetectionBox, MissionEvent, MissionReport, TargetRecord


class MissionStore:
    def __init__(self, runs_root: str | pathlib.Path = "runs", run_id: Optional[str] = None) -> None:
        self.runs_root = pathlib.Path(runs_root)
        self.runs_root.mkdir(parents=True, exist_ok=True)
        if run_id is None:
            ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d_%H%M%S")
            run_id = f"run_{ts}_{int(time.time() * 1000) % 1000:03d}"
        self.run_id = run_id
        self.run_dir = self.runs_root / self.run_id
        self.snapshots_dir = self.run_dir / "snapshots"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.snapshots_dir.mkdir(parents=True, exist_ok=True)

        self.events_path = self.run_dir / "events.jsonl"
        self.coverage_map_path = self.run_dir / "coverage_map.json"
        self.report_path = self.run_dir / "report.json"
        self.targets_db_json_path = self.run_dir / "targets_database.json"
        self.targets_db_csv_path = self.run_dir / "targets_database.csv"

        self._lock = threading.Lock()
        self.events: List[MissionEvent] = []

    def log_event(self, event_type: str, state: str, message: str, details: Optional[Dict[str, Any]] = None) -> MissionEvent:
        ev = MissionEvent(
            timestamp=round(time.time(), 4),
            event_type=event_type,
            state=state,
            message=message,
            details=details or {},
        )
        line = json.dumps(ev.to_dict()) + "\n"
        with self._lock:
            self.events.append(ev)
            with self.events_path.open("a", encoding="utf-8") as f:
                f.write(line)
                f.flush()
        return ev

    def save_snapshot(self, frame_bgr: np.ndarray, boxes: List[DetectionBox], label_prefix: str = "target", caption: str = "") -> str:
        annotated = frame_bgr.copy()
        for box in boxes:
            x1, y1, x2, y2 = int(box.x1), int(box.y1), int(box.x2), int(box.y2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 230, 64), 2)
            txt = f"PERSON {box.confidence:.2f}"
            cv2.putText(annotated, txt, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 230, 64), 2, cv2.LINE_AA)
        if caption:
            cv2.putText(annotated, caption, (10, annotated.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        fname = f"{label_prefix}_{int(time.time() * 1000)}.jpg"
        out_path = self.snapshots_dir / fname
        cv2.imwrite(str(out_path), annotated)
        return f"snapshots/{fname}"

    def save_targets_database(self, targets: List[TargetRecord]) -> pathlib.Path:
        payload = {
            "run_id": self.run_id,
            "updated_at": round(time.time(), 3),
            "total_targets": len(targets),
            "targets": [t.to_dict() for t in targets],
        }
        csv_text = self.format_targets_csv(targets)
        with self._lock:
            self.targets_db_json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            self.targets_db_csv_path.write_text(csv_text, encoding="utf-8")
        return self.targets_db_json_path

    @staticmethod
    def format_targets_csv(targets: List[TargetRecord]) -> str:
        buf = io.StringIO()
        writer = csv.writer(buf)
        writer.writerow([
            "target_id", "association_label", "triage_priority", "posture_label",
            "x_m", "y_m", "col", "row", "distance_from_home_m", "rescue_path_distance_m",
            "rescue_directions", "path_from_home", "confidence", "sightings_count",
            "first_seen_ts", "last_seen_ts", "snapshot_path"
        ])
        for t in targets:
            writer.writerow([
                t.target_id, t.association_label, t.triage_priority, t.posture_label,
                t.x_m, t.y_m, t.col, t.row, t.distance_from_home_m, t.rescue_path_distance_m,
                t.rescue_directions, json.dumps(t.path_from_home), t.confidence, t.sightings_count,
                round(t.first_seen_ts, 3), round(t.last_seen_ts, 3), t.snapshot_path or ""
            ])
        return buf.getvalue()

    def save_coverage_map(self, grid: ArenaGrid, trail: List[Any], planned_path: List[Any]) -> pathlib.Path:
        payload = grid.to_dict()
        payload["trail"] = [list(pt) for pt in trail]
        payload["planned_path"] = [list(pt) for pt in planned_path]
        payload["saved_at"] = round(time.time(), 3)
        with self._lock:
            self.coverage_map_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return self.coverage_map_path

    def save_report(self, report: MissionReport) -> pathlib.Path:
        with self._lock:
            self.report_path.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
        self.save_targets_database(report.targets)
        return self.report_path
