# AI-Assisted Search & Reconnaissance Rover (v5.0)

**Real Hardware First • Fixed Forward Camera • Bounded-Arena Coverage • Laptop-Based AI • Loophole-Hardened for 10-Day Build**

This repository contains the complete laptop control software, computer-vision pipeline, single-file web dashboard, mock hardware simulator, and dual-ESP32 firmware (`rover_control.ino` + `cam_stream.ino`) for a low-cost indoor AI-Assisted Search & Reconnaissance Rover.

---

## 1. System Overview & Core Architecture

* **Two-Board ESP32 Architecture**:
  * **Board 1 — ESP32-CAM (OV2640)**: Dedicated exclusively to fixed forward-facing MJPEG streaming (`http://<cam_ip>:81/stream`) in `CAMERA_GRAB_LATEST` mode (`fb_count = 2`, JPEG quality `12`). Zero pan/tilt servos are used; the rover changes its viewpoint by driving and stopping at scheduled observation points.
  * **Board 2 — ESP32 DevKit (30/38-pin)**: Dedicated to non-blocking `millis()` motor control (TB6612FNG preferred; L298N supported), timeout-guarded HC-SR04 ultrasonic obstacle stopping (`pulseIn(ECHO, HIGH, 15000UL)`), 1500 ms communication watchdog, idempotent command tracking, and latched E-Stop.
* **Bounded-Arena Coverage + Smart AI Target Confirmation**:
  * Systematic coverage over a known bounded indoor arena (default `3.0 m × 2.0 m` with solid walls $\ge 15\text{ cm}$ tall and `0.30 m` grid cells) using deterministic **Boustrophedon (serpentine)** order (open arena) or **nearest-cell A\*** order (maze/corridors).
  * **AI-Assisted Target-Confirmation Step**: When YOLO (`yolo11n.pt` / `yolov8n.pt`) detects a candidate person with borderline confidence (`0.22 <= conf < 0.35`) or a small distant bounding box (`25 px <= height < 36 px`), the planner dynamically schedules up to 1 confirmation step (a 1-cell approach along the detection bearing or an immediate in-place re-scan) before resuming serpentine coverage.
* **Honest Dual Coverage Reporting**:
  * **Visited Coverage (`visited_coverage_pct`)**: Percentage of eligible non-blocked cells the rover physically drove through and scanned.
  * **Observed Coverage (`observed_coverage_pct`)**: Percentage of eligible cells intersected by ray-casting the camera's $60^\circ$ horizontal FOV out to `2.0 m` during valid stationary scans (stopped by walls and obstacles). Visited coverage is never conflated with observed coverage, and 100% visited coverage is never claimed if any eligible cell remains `UNKNOWN`, `UNCERTAIN`, or `UNREACHABLE`.

---

## 2. Decision / Conflict Resolution (v5.0 Hardening)

Every timing, uncertainty, polling, and sensing parameter in v5.0 has been mathematically and electrically hardened to eliminate common physical hardware failures during a live demonstration:

| Failure Mode in Earlier Specs (v4.1) | v5.0 Engineering Fix | Implementation Location |
| :--- | :--- | :--- |
| **ESP32 TCP Socket Exhaustion** from 10–15 Hz HTTP polling | Idle heartbeat set to **2 Hz (`500 ms`)**, active move `/status` polling set to **4 Hz (`250 ms`)**, and `watchdog_ms` set to **`1500 ms`** ($3\times$ poll interval) so normal 2.4 GHz Wi-Fi jitter never aborts a move. Any valid `/status`, `/heartbeat`, or `/move` refreshes `last_comms_ms`. | `rover/robot.py`, `firmware/rover_control/rover_control.ino` |
| **Blocking Ultrasonic Jitter** (`pulseIn` default 1,000,000 µs timeout starves HTTP & motor timers) | Every ultrasonic read uses `pulseIn(PIN_US_ECHO, HIGH, 15000UL)` (**15 ms max timeout** $\approx 2.5\text{ m}$ range), sampled at most once every `85 ms` with a rolling 3-sample median filter and 2-consecutive-hit obstacle abort. | `firmware/rover_control/rover_control.ino` |
| **Monotonic Heading Trap** (`k_turn=5°`, `sigma_heading_max=20°` froze the mission after 4 turns) | Default `k_turn_deg = 2.5°` (`1.0°` with gyro), `sigma_heading_max_deg = 60.0°`, `sigma_pos_max_m = 0.80 m`, `home_tolerance_m = 0.40 m`, and configurable `uncertainty_policy` (`warn_only` default for live demo continuity, `return_home`, or `recovery_required`). | `config.yaml`, `rover/localization.py`, `rover/mission.py` |
| **Ultrasonic Specular Reflection Jumps** on angled wall approaches | Gated ultrasonic wall-snapping is disabled by default (`enable_wall_snap: false`); arena boundary clamping keeps $(x, y)$ strictly inside the physical perimeter walls while preserving `try_wall_snap()` behind a config flag for unit verification. | `rover/localization.py`, `tests/test_localization.py` |
| **Slow Scan Stops Draining Battery** (~2.5 s per scan stop) | Fast stop-and-scan timing (`~0.7 s` total dwell: `settle_s = 0.25 s`, `camera_latency_s = 0.20 s`, `scan_frames = 3`, confirmation $\ge 2\text{ of }3$ frames). | `config.yaml`, `rover/mission.py` |
| **ESP32-CAM `FIN_WAIT` Socket Lockup** on reconnect | `cam_stream.ino` enables `lru_purge_enable = true` and `CAMERA_GRAB_LATEST`; `MjpegFrameSource` sends `Connection: close`, explicitly closes the raw socket on stall, and waits `reconnect_cooldown_s = 1.0 s` before reconnecting. | `rover/frames.py`, `firmware/cam_stream/cam_stream.ino` |
| **ESP32 Arduino Core 2.x vs 3.x LEDC PWM Breakage** | `rover_control.ino` includes `#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)` dual compatibility macros so it compiles cleanly on both Core `2.0.14` and Core `3.x`. | `firmware/rover_control/rover_control.ino` |

---

## 3. Bill of Materials (BOM) & Pinout Table

### 3.1 Expected Hardware Components
* **ESP32-CAM (AI-Thinker OV2640)** + `ESP32-CAM-MB` USB programmer (or FTDI 3.3 V adapter).
* **ESP32 DevKit V1 (30-pin or 38-pin)** for motor and sensor control.
* **2WD Robot Chassis** with two geared DC TT motors, wheels, and front/rear caster.
* **Motor Driver**: **TB6612FNG** (strongly preferred for low MOSFET drop) or **L298N** (supported with identical logic pins).
* **HC-SR04 Ultrasonic Sensor** + $1\text{ k}\Omega$ and $2\text{ k}\Omega$ resistors for 5 V $\to$ 3.3 V Echo voltage divider.
* **Power Supplies**:
  * **Supply A (5 V Logic)**: 5 V USB power bank or 5 V / 3 A buck converter feeding ESP32 DevKit `5V/VIN` and ESP32-CAM `5V`.
  * **Supply B (Motors)**: 2S Li-ion (`7.4 V`) or 6× AA pack feeding motor driver `VM`/`12V` through a **physical master kill switch**.
  * **Decoupling**: $470\,\mu\text{F} - 1000\,\mu\text{F}$ electrolytic capacitor across ESP32-CAM `5V`/`GND` + $100\text{ nF}$ ceramic capacitors across motor terminals.
* **Optional P1 Hardware (Config-Switched, OFF by default)**: MPU6050 (`GY-521`) I2C gyro (`SDA=21`, `SCL=22`); front micro-switch bumper (`GPIO 4`).

### 3.2 ESP32 DevKit Pinout (`firmware/rover_control/rover_control.ino`)
*Verify physical board silkscreen labels before powering on:*

| Function | ESP32 DevKit GPIO | TB6612FNG Pin | L298N Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| Left Motor PWM | `GPIO 25` | `PWMA` | `ENA` | 20 kHz 8-bit LEDC PWM |
| Left Motor Dir 1 | `GPIO 26` | `AIN1` | `IN1` | Digital Output |
| Left Motor Dir 2 | `GPIO 27` | `AIN2` | `IN2` | Digital Output |
| Right Motor PWM | `GPIO 33` | `PWMB` | `ENB` | 20 kHz 8-bit LEDC PWM |
| Right Motor Dir 1 | `GPIO 14` | `BIN1` | `IN3` | Digital Output |
| Right Motor Dir 2 | `GPIO 12` | `BIN2` | `IN4` | Digital Output |
| Driver Standby | `GPIO 13` | `STBY` | *(Unconnected)* | Driven `HIGH` when active |
| Ultrasonic TRIG | `GPIO 32` | — | — | 10 µs trigger pulse |
| Ultrasonic ECHO | `GPIO 35` | — | — | **Input-only GPIO; requires 1 kΩ / 2 kΩ divider from 5 V ECHO!** |
| Front Bumper (Opt.) | `GPIO 4` | — | — | Active-LOW (`INPUT_PULLUP`), `enable_bumper: false` default |
| I2C Gyro (Opt.) | `GPIO 21 / 22` | — | — | MPU6050 `SDA=21`, `SCL=22`, `use_imu: false` default |

### 3.3 HC-SR04 5 V to 3.3 V Echo Voltage Divider
```text
HC-SR04 ECHO (5V) ───[ 1 kΩ ]───┬───> ESP32 DevKit GPIO 35 (3.3V safe)
                                │
                             [ 2 kΩ ]
                                │
                               GND (Common Ground across Logic + Motor supplies)
```

---

## 4. Hardware Protocol v1 Specification

Implemented identically in `firmware/rover_control/rover_control.ino`, `rover/robot.py`, and `tools/mock_esp32.py`:

* **`POST /config`**: Validates and applies `cm_per_s`, `deg_per_s`, `speed_pwm`, `turn_pwm`, `min_pwm`, `max_pwm`, `left_trim`, `right_trim`, `motor_invert_left`, `motor_invert_right`, `stop_distance_cm`, `watchdog_ms` (default `1500`), `max_move_cm`, `max_turn_deg`, `use_imu`. Returns `{accepted: true, fw_version, protocol: "v1", boot_id, capabilities}`. Firmware rejects `/move` with `reason: "not_configured"` until `/config` succeeds.
* **`POST /move`**: `{id: int, type: "forward"|"backward"|"turn", value: float}` $\to$ `{id, accepted, duplicate, state, reason?}`. Idempotent on `id == last_accepted_id`; rejects `id < last_accepted_id` with `stale_id`.
* **`GET /status`**: Returns `{boot_id, last_id, state, elapsed_ms, executed_cm, executed_deg, executed_source, ultrasonic_cm, heading_deg, imu_ok, bumper, rssi, uptime_ms, fw_version, protocol}` and refreshes `last_comms_ms`.
* **`POST /stop`**: Immediate motor stop; transitions active move to `aborted_stop` and records partial `executed_cm` / `executed_deg`.
* **`POST /estop` & `POST /estop_reset`**: Latches / unlatches hardware `estopped` state.
* **`POST /heartbeat`**: Refreshes watchdog and returns `{ok: true, boot_id, state, ultrasonic_cm}`.

---

## 5. Windows 11 PowerShell Setup & Commands

### 5.1 Environment & OpenCV Conflict Resolution
On Windows 11 PowerShell (Python 3.11):
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt
```
**OpenCV Package Conflict Resolution**: Never install both `opencv-python` and `opencv-contrib-python` in the same virtual environment, as they overwrite the same `cv2` namespace. If `cv2` raises an import error, run:
```powershell
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless
pip install opencv-contrib-python>=4.9.0
```
**NVIDIA RTX 2050 (4 GB VRAM) CUDA Setup**: `YoloDetector` automatically selects CUDA if PyTorch with CUDA is installed, and falls back cleanly to CPU otherwise.

### 5.2 Exact Supported Commands
```powershell
# 1. Run full live hardware server (connects to ESP32 DevKit + ESP32-CAM)
python -m rover.main hardware

# 2. Run full mock mode (starts local MockEsp32Server + synthetic stream + dashboard on http://localhost:8000)
python -m rover.main mock

# 3. Run standalone Mock ESP32 DevKit HTTP server on http://127.0.0.1:8081
python tools/mock_esp32.py

# 4. Run 10-minute interactive CLI calibration wizard
python -m rover.main calibrate

# 5. Test YOLO person detection on webcam or video file (with optional ESP32-CAM degradation)
python -m rover.main yolo-test --source webcam
python -m rover.main yolo-test --source file.mp4 --degrade

# 6. Run automated test suite
pytest -q
```

---

## 6. 10-Day Hardware Bring-Up & Acceptance Checklist

1. **Power & Wiring Check**: Verify ESP32 DevKit & ESP32-CAM pinouts, common ground, 5 V logic supply, motor supply with physical kill switch, $470\,\mu\text{F}$ decoupling capacitor, and HC-SR04 $1\text{ k}\Omega / 2\text{ k}\Omega$ Echo divider.
2. **Wheels-Off-Ground Motor Test**: Verify forward/backward/left/right motor directions and save `motor_invert_left` / `motor_invert_right` in `calibration.json`.
3. **Watchdog Cutoff Test**: Disconnect Wi-Fi or pause client during a move; verify motors stop within `1.5 s` and `/status` reports `aborted_watchdog` with partial `executed_cm`.
4. **Ultrasonic Check**: Verify stable readings at `20 cm`, `50 cm`, and `100 cm` with zero HTTP blocking.
5. **Camera Stream Check**: Verify OV2640 lens focus at `1.5–2.0 m` and stable MJPEG streaming ($\ge 6\text{ FPS}$) through `http://localhost:8000/video`.
6. **10-Minute Calibration**: Run the `3.0 s` straight-line test and `90°` turn test via `python -m rover.main calibrate` or the dashboard Setup tab; verify $< 10\text{ cm}$ straight error per meter and $< 8^\circ$ turn error.
7. **Obstacle Stop & A\* Replan**: Verify the rover halts before a solid box at `20 cm` in 5/5 runs, marks the cell `BLOCKED`, and detours via A\*.
8. **Person Detection & Smart AI Confirmation**: Verify standing/seated person detection at `1.0–2.5 m` (2-of-3 frame confirmation) and verify that borderline sightings trigger the 1-step AI confirmation approach.
9. **Full Bounded-Arena Run**: Verify serpentine coverage, separate Visited % vs Observed % metrics, Return-Home within `0.40 m` tolerance, and generated artifacts under `runs/<run_id>/` (`events.jsonl`, `coverage_map.json`, `report.json`, and `snapshots/`).
