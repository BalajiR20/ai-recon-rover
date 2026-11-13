# Complete Step-by-Step Hardware & Software Build Manual (v5.5 Fixed Edition)

**AI-Assisted Search & Reconnaissance Rover — Single ESP32 DevKit V1 + Smartphone Camera + TB6612FNG + HC-SR04**

---

## 1. Zero-Burn Safety Checks (Read Before Connecting Anything!)

1. **Keep 6V Motor Power (VM) Strictly Separate from 3.3V Logic (VCC)**:
   * The 4×AA Battery Red (+) wire connects **ONLY to TB6612 VM** (via the power switch).
   * The TB6612 **VCC** pin connects **ONLY to ESP32 3V3** (orange wire).
   * Never let 6V touch the ESP32 3V3 or GPIO pins!
2. **Always Use the 1 kΩ + 2 kΩ Resistor Voltage Divider on ECHO**:
   * HC-SR04 runs at 5V (from ESP32 VIN), so its ECHO pin outputs 5.0V.
   * Passing ECHO through **1 kΩ in series into ESP32 D35**, with **2 kΩ connected to GND**, steps the pulse down to a safe **3.33V**.
3. **Common Ground (GND) is Mandatory**:
   * ESP32 GND, TB6612 GND, 4×AA Battery Black (-), and HC-SR04 GND must all connect to the same Breadboard Ground Rail.

---

## 2. Exact Wire-by-Wire Pinout Table

| Wire # | From Pin | To Pin | Wire Color & Purpose |
| :---: | :--- | :--- | :--- |
| **W1** | ESP32 `GND` (Left Pin 14) | Breadboard Common GND Rail | **Black** — Common Ground reference |
| **W2** | TB6612 `GND` (Pins 3 & 8) | Breadboard Common GND Rail | **Black** — Common Ground reference |
| **W3** | 4×AA Battery `Black (-)` | Breadboard Common GND Rail | **Black** — Common Ground reference |
| **W4** | HC-SR04 `GND` & 2kΩ Leg | Breadboard Common GND Rail | **Black** — Common Ground reference |
| **W5** | ESP32 `3V3` (Right Pin 15) | TB6612 `VCC` (Pin 2) | **Orange** — 3.3V Logic Supply |
| **W6** | 4×AA Battery `Red (+)` | Switch &rarr; TB6612 `VM` (Pin 1) | **Red** — 6.0V Motor Power |
| **W7** | ESP32 `D13` (Left Pin 13) | TB6612 `STBY` (Right Pin 4) | **Green** — Motor Driver Standby Enable |
| **W8** | ESP32 `D25` (Left Pin 8) | TB6612 `PWMA` (Right Pin 1) | **Yellow** — Left Motor PWM Speed |
| **W9** | ESP32 `D26` (Left Pin 9) | TB6612 `AIN1` (Right Pin 3) | **Light Blue** — Left Motor Direction 1 |
| **W10** | ESP32 `D27` (Left Pin 10) | TB6612 `AIN2` (Right Pin 2) | **Blue** — Left Motor Direction 2 |
| **W11** | ESP32 `D33` (Left Pin 7) | TB6612 `PWMB` (Right Pin 7) | **Yellow** — Right Motor PWM Speed |
| **W12** | ESP32 `D14` (Left Pin 11) | TB6612 `BIN1` (Right Pin 5) | **Light Blue** — Right Motor Direction 1 |
| **W13** | ESP32 `D12` (Left Pin 12) | TB6612 `BIN2` (Right Pin 6) | **Blue** — Right Motor Direction 2 *(Safe fallback: D19)* |
| **W14/15** | TB6612 `AO1` & `AO2` | Left TT Gear Motor Terminals | **Cyan** — Left Wheel Drive |
| **W16/17** | TB6612 `BO1` & `BO2` | Right TT Gear Motor Terminals | **Cyan** — Right Wheel Drive |
| **W18** | ESP32 `VIN` (Left Pin 15) | HC-SR04 `VCC` | **Bright Red** — 5.0V Sensor Power |
| **W19** | ESP32 `D32` (Left Pin 6) | HC-SR04 `TRIG` | **Purple** — Ultrasonic Trigger (10 µs) |
| **W20** | HC-SR04 `ECHO` &rarr; 1kΩ | Midpoint Node &rarr; ESP32 `D35` | **Lavender** — Safe 3.33V Echo Pulse |

---

## 3. Safe 4-Step Bring-Up Protocol

1. **Step 1 — Bare ESP32 Flash (0 Wires Attached)**:
   * Connect bare ESP32 to laptop via USB.
   * Open `firmware/rover_control/rover_control.ino` in Arduino IDE. Set Wi-Fi SSID to `ROVER_NET` and Password to `roverpass123`.
   * Upload (Board: DOIT ESP32 DEVKIT V1). Open Serial Monitor at 115200 baud.
   * Verify it connects and prints its IP (e.g. `192.168.43.50`). Unplug USB.
2. **Step 2 — Test Ultrasonic Sensor Only**:
   * Wire only the HC-SR04 and 1kΩ/2kΩ divider (Wires W1, W4, W18, W19, W20).
   * Plug USB back in. Open `http://<ESP32_IP>/status` in browser. Confirm `ultrasonic_cm` reads accurately. Unplug USB.
3. **Step 3 — Wire TB6612FNG (Batteries OUT of Holder)**:
   * Wire the TB6612FNG and motors. Double-check that Battery Red is on `VM` and ESP32 3V3 is on `VCC`.
4. **Step 4 — Prop Wheels Up & Test Drive**:
   * Elevate rover so wheels hang freely in air.
   * Plug ESP32 into phone/power bank, insert 4 AA batteries, switch ON.
   * Start `python -m rover.main` on laptop, open `http://localhost:8000`, and click `▲ Fwd 15 cm`. Both wheels should spin forward!
