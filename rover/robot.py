"""HTTP client interface for ESP32 hardware motor & sensor controller."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional
import requests

from rover.config import CalibrationData, RobotConfig
from rover.models import RobotStatus


class RobotCommunicationError(RuntimeError):
    pass


class RobotInterface:
    last_status: RobotStatus

    def configure(self, cal: CalibrationData) -> Dict[str, Any]:
        raise NotImplementedError

    def get_status(self) -> RobotStatus:
        raise NotImplementedError

    def execute_move(self, move_type: str, value: float) -> RobotStatus:
        raise NotImplementedError

    def stop(self) -> RobotStatus:
        raise NotImplementedError

    def estop(self) -> RobotStatus:
        raise NotImplementedError

    def reset_estop(self) -> RobotStatus:
        raise NotImplementedError


class Esp32Robot(RobotInterface):
    def __init__(
        self,
        base_url: str,
        robot_cfg: Optional[RobotConfig] = None,
        cal: Optional[CalibrationData] = None,
        start_background_worker: bool = False,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.robot_cfg = robot_cfg or RobotConfig()
        self.cal = cal or CalibrationData()
        self.session = requests.Session()
        self.session.headers.update({"Connection": "close"})
        self._http_lock = threading.RLock()
        self._moving_flag = False
        self._last_status_ts = 0.0
        self._cmd_counter = 100
        self.last_status = RobotStatus(connected=False)
        self.reboot_detected = False
        self._last_boot_id = 0

    def set_base_url(self, new_url: str) -> None:
        with self._http_lock:
            self.base_url = new_url.strip().rstrip("/")

    def _effective_timeout(self) -> float:
        if "127.0.0.1" in self.base_url or "localhost" in self.base_url:
            return max(1.0, float(self.robot_cfg.http_timeout_s))
        return max(2.5, float(self.robot_cfg.http_timeout_s))

    def _request(
        self,
        method: str,
        path: str,
        json_payload: Optional[Dict[str, Any]] = None,
        retries: int = 2,
        timeout: Optional[float] = None,
    ) -> requests.Response:
        url = f"{self.base_url}{path}"
        eff_timeout = timeout if timeout is not None else self._effective_timeout()
        last_exc: Optional[Exception] = None
        for attempt in range(retries + 1):
            try:
                with self._http_lock:
                    resp = self.session.request(
                        method=method,
                        url=url,
                        json=json_payload,
                        headers={"Connection": "close"},
                        timeout=eff_timeout,
                    )
                resp.raise_for_status()
                return resp
            except Exception as exc:
                last_exc = exc
                if attempt < retries:
                    time.sleep(0.18 * (attempt + 1))
        raise RobotCommunicationError(f"{method} {url} failed: {last_exc}") from last_exc

    def configure(self, cal: CalibrationData) -> Dict[str, Any]:
        self.cal = cal
        try:
            resp = self._request("POST", "/config", json_payload=cal.to_dict(), retries=2)
            return resp.json()
        except Exception as exc:
            raise RobotCommunicationError(f"Failed to configure ESP32: {exc}") from exc

    def get_status(self) -> RobotStatus:
        try:
            resp = self._request("GET", "/status", retries=1)
            data = resp.json()
            st = RobotStatus(**data)
            st.connected = True
            if self._last_boot_id != 0 and st.boot_id != self._last_boot_id:
                self.reboot_detected = True
            self._last_boot_id = st.boot_id
            self.last_status = st
            self._last_status_ts = time.time()
            return st
        except Exception as exc:
            self.last_status.connected = False
            raise RobotCommunicationError(f"ESP32 status comms error: {exc}") from exc

    def refresh_status_if_stale(self, max_age_s: float = 1.5) -> RobotStatus:
        """Non-blocking status refresh for idle telemetry; never collides with active moves."""
        if self._moving_flag or (time.time() - self._last_status_ts < max_age_s):
            return self.last_status
        acquired = self._http_lock.acquire(blocking=False)
        if not acquired:
            return self.last_status
        try:
            resp = self.session.get(
                f"{self.base_url}/status",
                headers={"Connection": "close"},
                timeout=1.2,
            )
            if resp.status_code == 200:
                data = resp.json()
                st = RobotStatus(**data)
                st.connected = True
                self.last_status = st
            self._last_status_ts = time.time()
        except Exception:
            self.last_status.connected = False
            self._last_status_ts = time.time()
        finally:
            self._http_lock.release()
        return self.last_status

    def execute_move(self, move_type: str, value: float) -> RobotStatus:
        self._cmd_counter += 1
        cid = self._cmd_counter
        eff_turn_rate = self.cal.effective_deg_per_s(value) if hasattr(self.cal, "effective_deg_per_s") else max(1.0, self.cal.deg_per_s)
        hw_value = value
        if move_type == "turn" and eff_turn_rate > 1.0 and self.cal.deg_per_s > 1.0:
            hw_value = round(value * (self.cal.deg_per_s / eff_turn_rate), 2)
        payload = {"id": cid, "type": move_type, "value": hw_value}
        self._moving_flag = True
        try:
            try:
                self._request("POST", "/move", json_payload=payload, retries=2)
            except Exception as exc:
                raise RobotCommunicationError(f"Move command {cid} failed: {exc}") from exc

            poll_interval = 1.0 / max(1.0, self.robot_cfg.active_poll_hz)
            est_duration_s = (
                (abs(value) / max(0.1, self.cal.cm_per_s))
                if move_type != "turn"
                else (abs(value) / max(1.0, eff_turn_rate))
            )
            start_t = time.time()
            deadline = start_t + est_duration_s + 3.5

            # Give ESP32 soft-start motor ramp a brief settle window before polling
            settle_wait = min(0.22, max(0.02, poll_interval))
            time.sleep(settle_wait)

            while time.time() < deadline:
                try:
                    st = self.get_status()
                    if st.last_id == cid and st.state == "idle":
                        st.executed_cm = abs(value) if move_type != "turn" else 0.0
                        st.executed_deg = value if move_type == "turn" else 0.0
                        return st
                    if st.state in ("estopped", "aborted_obstacle", "aborted_watchdog", "aborted_stop"):
                        return st
                except RobotCommunicationError:
                    # Tolerate transient Wi-Fi jitter while motors are spinning;
                    # wait until estimated physical move duration finishes.
                    elapsed = time.time() - start_t
                    if elapsed >= est_duration_s + 0.35:
                        break
                time.sleep(poll_interval)

            # Final status check after move completion window
            try:
                final_st = self.get_status()
                if final_st.state == "idle":
                    final_st.executed_cm = abs(value) if move_type != "turn" else 0.0
                    final_st.executed_deg = value if move_type == "turn" else 0.0
                return final_st
            except RobotCommunicationError:
                # Fallback dead-reckoning status if ESP32 Wi-Fi is still re-associating
                fallback = RobotStatus(
                    boot_id=self._last_boot_id,
                    last_id=cid,
                    state="idle",
                    last_executed_state="done_estimated",
                    executed_cm=abs(value) if move_type != "turn" else 0.0,
                    executed_deg=value if move_type == "turn" else 0.0,
                    ultrasonic_cm=self.last_status.ultrasonic_cm,
                    rssi=self.last_status.rssi,
                    connected=False,
                )
                self.last_status = fallback
                return fallback
        finally:
            self._moving_flag = False

    def stop(self) -> RobotStatus:
        try:
            self._request("POST", "/stop", retries=1, timeout=1.5)
            return self.get_status()
        except Exception:
            return self.last_status

    def estop(self) -> RobotStatus:
        try:
            self._request("POST", "/estop", retries=1, timeout=1.5)
            return self.get_status()
        except Exception:
            return self.last_status

    def reset_estop(self) -> RobotStatus:
        try:
            self._request("POST", "/estop_reset", retries=1, timeout=1.5)
            return self.get_status()
        except Exception:
            return self.last_status

    def close(self) -> None:
        with self._http_lock:
            self.session.close()
