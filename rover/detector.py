"""YOLO Pose/Person detector (yolov8n-pose.pt / yolov8n.pt), HSV Semantic Ground Hazard detector, and Farneback Optical Flow."""
from __future__ import annotations

import math
import pathlib
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
import cv2
import numpy as np

from rover.config import VisionConfig
from rover.models import DetectionBox


def detect_ground_hazards(frame_bgr: np.ndarray, min_area_px: float = 1800.0) -> List[Dict[str, Any]]:
    """
    Lightweight HSV color-segmentation detector watching the lower 45% of the frame
    for bright red/orange ground hazard markers (e.g., red A4 paper or hazard tape).
    Runs in <2 ms on CPU with zero deep-learning overhead.
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []
    h, w = frame_bgr.shape[:2]
    y_start = int(h * 0.55)
    roi = frame_bgr[y_start:h, 0:w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red wraps around 0 and 180 in OpenCV HSV
    mask1 = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([168, 120, 80]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hazards: List[Dict[str, Any]] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= min_area_px:
            x, y, bw, bh = cv2.boundingRect(cnt)
            hazards.append({
                "x1": int(x),
                "y1": int(y + y_start),
                "x2": int(x + bw),
                "y2": int(y + y_start + bh),
                "area_px": round(float(area), 1),
                "label": "THERMAL / GROUND HAZARD",
            })
    return hazards


def compute_optical_flow_shift(
    prev_bgr: Optional[np.ndarray],
    curr_bgr: Optional[np.ndarray],
) -> Tuple[float, float, float]:
    """
    Compute Farneback dense optical flow on downscaled (160x120) grayscale frames
    between pre-move and post-move snapshots (~2-4 ms) to verify physical motion vs wheel slip.
    Returns (mean_dx_px, mean_dy_px, mean_magnitude_px).
    """
    if prev_bgr is None or curr_bgr is None:
        return (0.0, 0.0, 0.0)
    try:
        g1 = cv2.cvtColor(cv2.resize(prev_bgr, (160, 120)), cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(cv2.resize(curr_bgr, (160, 120)), cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(
            g1, g2, None, pyr_scale=0.5, levels=2, winsize=11, iterations=2, poly_n=5, poly_sigma=1.1, flags=0
        )
        dx = float(np.median(flow[..., 0]))
        dy = float(np.median(flow[..., 1]))
        mag = float(np.mean(np.hypot(flow[..., 0], flow[..., 1])))
        return (round(dx, 2), round(dy, 2), round(mag, 2))
    except Exception:
        return (0.0, 0.0, 0.0)


def estimate_visual_heading_drift_deg(
    prev_bgr: Optional[np.ndarray],
    curr_bgr: Optional[np.ndarray],
    hfov_deg: float = 60.0,
) -> Tuple[float, float, float, float]:
    """
    Visual Compass: Estimate unintended horizontal yaw drift (in degrees) during a straight forward move
    by fusing Fourier Phase Correlation (global sub-pixel horizon shift) with Farneback Optical Flow
    in the horizon/mid-ground band (y: 15%..70%).
    Returns (dx_px, dy_px, mag_px, drift_deg).
    Positive drift_deg means rover veered Left (+CCW); negative means rover veered Right (-CW).
    """
    if prev_bgr is None or curr_bgr is None:
        return (0.0, 0.0, 0.0, 0.0)
    try:
        g1_full = cv2.cvtColor(cv2.resize(prev_bgr, (160, 120)), cv2.COLOR_BGR2GRAY)
        g2_full = cv2.cvtColor(cv2.resize(curr_bgr, (160, 120)), cv2.COLOR_BGR2GRAY)
        # Focus on horizon/wall band (rows 18..84 of 120) to reject near-field floor tile distortion
        g1 = g1_full[18:84, :]
        g2 = g2_full[18:84, :]

        # 1. Global Fourier Phase Correlation on horizon strip (captures wider yaw shifts cleanly)
        (shift_x, shift_y), response = cv2.phaseCorrelate(np.float32(g1), np.float32(g2))

        # 2. Dense Farneback Optical Flow for local motion confidence
        flow = cv2.calcOpticalFlowFarneback(
            g1, g2, None, pyr_scale=0.5, levels=3, winsize=15, iterations=3, poly_n=5, poly_sigma=1.2, flags=0
        )
        mag_map = np.hypot(flow[..., 0], flow[..., 1])
        active = mag_map >= 0.35
        if not np.any(active) and abs(shift_x) < 0.8:
            return (0.0, 0.0, 0.0, 0.0)

        fb_dx = float(np.median(flow[..., 0][active])) if np.any(active) else 0.0
        fb_dy = float(np.median(flow[..., 1][active])) if np.any(active) else 0.0
        fb_mag = float(np.mean(mag_map[active])) if np.any(active) else 0.0

        # Fuse phase correlation shift_x with Farneback dx
        dx = float(shift_x) if (response >= 0.12 and abs(shift_x) >= abs(fb_dx)) else fb_dx
        dy = float(shift_y) if response >= 0.12 else fb_dy
        mag = max(fb_mag, float(math.hypot(dx, dy)))
        if mag < 0.50:
            return (round(dx, 2), round(dy, 2), round(mag, 2), 0.0)

        raw_drift = dx * (hfov_deg / 160.0)
        drift_deg = max(-24.0, min(24.0, raw_drift))
        return (round(dx, 2), round(dy, 2), round(mag, 2), round(drift_deg, 2))
    except Exception:
        return (0.0, 0.0, 0.0, 0.0)


class PersonDetector:
    is_loaded: bool = True
    device_name: str = "CPU"

    def detect(self, frame_bgr: np.ndarray, frame_index: int = 0) -> List[DetectionBox]:
        raise NotImplementedError


class YoloDetector(PersonDetector):
    def __init__(self, vision_cfg: Optional[VisionConfig] = None, allow_mock_fallback: bool = True) -> None:
        self.vision_cfg = vision_cfg or VisionConfig()
        self.allow_mock_fallback = allow_mock_fallback
        self._model = None
        self.is_loaded = False
        self.device_name = "CPU"
        self._load_model()

    def _load_model(self) -> None:
        try:
            import torch
            from ultralytics import YOLO

            if self.vision_cfg.device == "auto":
                self.device_name = "CUDA" if torch.cuda.is_available() else "CPU"
            else:
                self.device_name = self.vision_cfg.device.upper()

            primary = pathlib.Path(self.vision_cfg.model_path)
            fallback = pathlib.Path(self.vision_cfg.fallback_model_path)
            primary.parent.mkdir(parents=True, exist_ok=True)

            candidates = []
            if primary.exists():
                candidates.append(str(primary))
            if fallback.exists():
                candidates.append(str(fallback))
            candidates.extend([str(primary), "yolov8n-pose.pt", "yolov8n.pt"])

            for weights in candidates:
                try:
                    self._model = YOLO(weights)
                    self.is_loaded = True
                    return
                except Exception:
                    continue
            raise RuntimeError("Could not load any YOLO weights.")
        except Exception:
            if self.allow_mock_fallback:
                self._model = None
                self.device_name = "CPU-MOCK"
                self.is_loaded = True
            else:
                self.is_loaded = False

    def detect(self, frame_bgr: np.ndarray, frame_index: int = 0) -> List[DetectionBox]:
        if self._model is None:
            return []
        min_conf = min(self.vision_cfg.tentative_conf_min, self.vision_cfg.confirm_conf_min)
        device_arg = 0 if self.device_name == "CUDA" else "cpu"
        results = self._model.predict(
            source=frame_bgr,
            imgsz=self.vision_cfg.imgsz,
            conf=min_conf,
            classes=[0],
            device=device_arg,
            verbose=False,
        )
        boxes: List[DetectionBox] = []
        now = time.time()
        for res in results:
            if res.boxes is None:
                continue
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss = res.boxes.cls.cpu().numpy()

            kpts_data = None
            if getattr(res, "keypoints", None) is not None and res.keypoints.data is not None:
                kpts_data = res.keypoints.data.cpu().numpy()

            for idx, ((x1, y1, x2, y2), conf, cls_id) in enumerate(zip(xyxy, confs, clss)):
                if int(cls_id) != 0:
                    continue
                kpts_list: List[List[float]] = []
                if kpts_data is not None and idx < len(kpts_data):
                    kpts_list = [[round(float(kx), 1), round(float(ky), 1), round(float(kc), 3)] for kx, ky, kc in kpts_data[idx]]

                boxes.append(
                    DetectionBox(
                        x1=float(x1),
                        y1=float(y1),
                        x2=float(x2),
                        y2=float(y2),
                        confidence=float(conf),
                        class_id=0,
                        frame_index=frame_index,
                        timestamp=now,
                        keypoints=kpts_list,
                    )
                )
        return boxes


class SyntheticPersonDetector(PersonDetector):
    def __init__(self, scripted_fn: Optional[Callable[[int], List[DetectionBox]]] = None, device_name: str = "CPU-MOCK") -> None:
        self.scripted_fn = scripted_fn
        self.device_name = device_name
        self.is_loaded = True
        self.static_boxes: List[DetectionBox] = []

    def detect(self, frame_bgr: np.ndarray, frame_index: int = 0) -> List[DetectionBox]:
        if self.scripted_fn is not None:
            return self.scripted_fn(frame_index)
        now = time.time()
        return [
            DetectionBox(
                x1=b.x1,
                y1=b.y1,
                x2=b.x2,
                y2=b.y2,
                confidence=b.confidence,
                class_id=0,
                frame_index=frame_index,
                timestamp=now,
                keypoints=list(b.keypoints) if b.keypoints else [],
            )
            for b in self.static_boxes
        ]
