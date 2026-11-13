"""Threaded MJPEG camera stream reader with zero-lag latest-frame buffer."""
from __future__ import annotations

import threading
import time
from typing import Optional, Tuple
import cv2
import numpy as np
import requests

from rover.config import CameraConfig


class FrameSource:
    fps: float = 0.0

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], float, int]:
        raise NotImplementedError

    def frame_age_s(self) -> float:
        raise NotImplementedError

    def is_healthy(self, max_age_s: float = 1.5) -> bool:
        raise NotImplementedError


class MjpegFrameSource(FrameSource):
    def __init__(self, cam_cfg: Optional[CameraConfig] = None) -> None:
        self.cam_cfg = cam_cfg or CameraConfig()
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._frame_seq: int = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.reconnect_count: int = 0
        self.fps: float = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, name="MjpegFrameReader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], float, int]:
        with self._lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None
            return frame, self._latest_ts, self._frame_seq

    def frame_age_s(self) -> float:
        with self._lock:
            ts = self._latest_ts
            frame = self._latest_frame
        if frame is None or ts <= 0.0:
            return 999.0
        return max(0.0, time.time() - ts)

    def is_healthy(self, max_age_s: float = 1.5) -> bool:
        return self.frame_age_s() <= max_age_s

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            resp = None
            session = requests.Session()
            try:
                resp = session.get(
                    self.cam_cfg.stream_url,
                    headers={"Connection": "close"},
                    stream=True,
                    timeout=self.cam_cfg.stall_timeout_s,
                )
                resp.raise_for_status()
                buf = bytearray()
                last_fps_ts = time.time()
                frames_in_window = 0

                for chunk in resp.iter_content(chunk_size=4096):
                    if self._stop_event.is_set():
                        break
                    if not chunk:
                        continue
                    buf.extend(chunk)
                    start_idx = buf.find(b"\xff\xd8")
                    end_idx = buf.find(b"\xff\xd9", start_idx + 2) if start_idx != -1 else -1
                    while start_idx != -1 and end_idx != -1:
                        jpg_bytes = bytes(buf[start_idx : end_idx + 2])
                        del buf[: end_idx + 2]
                        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            now = time.time()
                            frames_in_window += 1
                            if now - last_fps_ts >= 1.0:
                                self.fps = round(frames_in_window / (now - last_fps_ts), 1)
                                frames_in_window = 0
                                last_fps_ts = now
                            with self._lock:
                                self._latest_frame = img
                                self._latest_ts = now
                                self._frame_seq += 1
                        start_idx = buf.find(b"\xff\xd8")
                        end_idx = buf.find(b"\xff\xd9", start_idx + 2) if start_idx != -1 else -1
                    if len(buf) > 512 * 1024:
                        buf.clear()
            except Exception:
                self.reconnect_count += 1
            finally:
                try:
                    if resp is not None:
                        resp.close()
                    session.close()
                except Exception:
                    pass
            time.sleep(self.cam_cfg.reconnect_cooldown_s)


class SyntheticFrameSource(FrameSource):
    def __init__(self, target_fps: float = 15.0) -> None:
        self.target_fps = target_fps
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._frame_seq: int = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.fps = target_fps

    def start(self) -> None:
        self._stop_event.clear()
        # Populate initial frame synchronously so preflight health checks never race thread startup on Windows
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "SYNTHETIC MOCK FRAME", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 120), 2)
        with self._lock:
            self._latest_frame = img
            self._latest_ts = time.time()
            self._frame_seq += 1
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        interval = 1.0 / max(1.0, self.target_fps)
        while not self._stop_event.is_set():
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, "SYNTHETIC MOCK FRAME", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 120), 2)
            with self._lock:
                self._latest_frame = img
                self._latest_ts = time.time()
                self._frame_seq += 1
            time.sleep(interval)

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], float, int]:
        with self._lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None
            return frame, self._latest_ts, self._frame_seq

    def frame_age_s(self) -> float:
        with self._lock:
            return max(0.0, time.time() - self._latest_ts) if self._latest_ts > 0 else 999.0

    def is_healthy(self, max_age_s: float = 1.5) -> bool:
        return self.frame_age_s() <= max_age_s
