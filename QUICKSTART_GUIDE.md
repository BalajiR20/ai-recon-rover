# AI-Assisted Search & Reconnaissance Rover — Quickstart Guide (v5.1 Mobile Edition)

**Single ESP32 DevKit V1 + Android Smartphone Camera + TB6612FNG + HC-SR04**

---

## 1. Today's 1-Day Build Timeline (1:00 PM – 11:00 PM)

You have ~10 hours, which is plenty of time if you follow this sequential schedule:

| Time Slot | Phase | Tasks | Target Outcome |
| :--- | :--- | :--- | :--- |
| **1:00 PM – 2:30 PM** | **Mechanical Build** | Assemble acrylic chassis, attach 2 TT motors, wheels, rear caster, and phone holder. | Sturdy rolling chassis. |
| **2:30 PM – 4:00 PM** | **Electrical Wiring** | Mount breadboard, TB6612FNG, ESP32, and HC-SR04 with 1k/2k resistors. Connect battery and power bank. | Clean, common-ground circuit. |
| **4:00 PM – 5:30 PM** | **ESP32 Firmware** | Flash `rover_control.ino` via Arduino IDE. Test `/status` and `/move` from browser. | Motors and ultrasonic responding. |
| **5:30 PM – 6:30 PM** | **Camera Stream** | Start hotspot on phone, launch IP Webcam app, verify 640×480 stream on laptop. | Video stream live in browser. |
| **6:30 PM – 8:00 PM** | **Arena Setup & Calib** | Lay out cardboard perimeter (2.0m × 1.5m). Run motor calibration script. | Verified speed (cm/s) & turns (deg/s). |
| **8:00 PM – 9:30 PM** | **Full Mission Run** | Launch `python -m rover.main`. Run autonomous sweep and YOLO target detection. | Mission complete, target logged on map. |
| **9:30 PM – 11:00 PM** | **Video & Polish** | Record clean 2-3 minute demonstration video from above; prepare presentation talking points. | Deliverable finished and ready for submission. |

---

## 2. Complete Wiring Table (Only 1 ESP32 Board!)

> **CRITICAL RULE**: Connect the **negative (black) wire of the 4×AA battery holder**, the **TB6612 GND pin**, and the **ESP32 GND pin** together to the same breadboard ground rail. Without this common ground, the motors will twitch or ignore commands!

### TB6612FNG Motor Driver Connections
* `VM` $\to$ 4×AA Battery (+) Red Wire (~6.0V via toggle switch)
* `VCC` $\to$ ESP32 `3V3` pin (3.3V logic supply)
* `GND` $\to$ Breadboard Common Ground Rail & Battery (-) Black Wire
* `STBY` $\to$ ESP32 `GPIO 13` (Software Standby enable)
* `PWMA` $\to$ ESP32 `GPIO 25` (Left Motor Speed)
* `AIN1` $\to$ ESP32 `GPIO 26` (Left Motor Direction 1)
* `AIN2` $\to$ ESP32 `GPIO 27` (Left Motor Direction 2)
* `PWMB` $\to$ ESP32 `GPIO 33` (Right Motor Speed)
* `BIN1` $\to$ ESP32 `GPIO 14` (Right Motor Direction 1)
* `BIN2` $\to$ ESP32 `GPIO 12` (Right Motor Direction 2)
* `AO1` & `AO2` $\to$ Left TT Gear Motor Terminals
* `BO1` & `BO2` $\to$ Right TT Gear Motor Terminals

### HC-SR04 Ultrasonic Sensor Connections
* `VCC` $\to$ ESP32 `VIN` pin (5V from USB Power Bank)
* `GND` $\to$ Breadboard Common Ground Rail
* `TRIG` $\to$ ESP32 `GPIO 32`
* `ECHO` $\to$ **1 kΩ resistor**. The other side of the 1 kΩ resistor connects to:
  * ESP32 `GPIO 35` (Input only)
  * **2 kΩ resistor** going to Common Ground Rail
  *(This steps down the 5V Echo pulse to a safe 3.3V for the ESP32).*

### ESP32 Logic Power
* Plug your USB Power Bank directly into the ESP32 DevKit Micro-USB port.

---

## 3. Phone & Hotspot Setup

1. **Enable Phone Hotspot**:
   * Hotspot Name (SSID): `ROVER_NET`
   * Password: `roverpass123`
   * **AP Band**: Must be set to **2.4 GHz** (ESP32 cannot see 5 GHz networks).
2. **Install & Launch IP Webcam**:
   * Open **IP Webcam** on your Android phone.
   * Under **Video preferences**:
     * Video resolution: `640x480`
     * Quality: `50%`
   * Scroll down and tap **Start server**.
   * Note the URL displayed on screen (e.g., `http://192.168.43.1:8080`).
   * Your stream URL is: `http://192.168.43.1:8080/video`.

---

## 4. Flashing the ESP32 Firmware

1. Connect your ESP32 DevKit to your laptop via USB cable.
2. Open Arduino IDE and open `firmware/rover_control/rover_control.ino`.
3. Check the Wi-Fi credentials near lines 25–28:
   ```cpp
   #define WIFI_SSID "ROVER_NET"
   #define WIFI_PASS "roverpass123"
   ```
4. Select Tools $\to$ Board $\to$ **DOIT ESP32 DEVKIT V1** (or ESP32 Dev Module).
5. Select the correct **COM Port** and click **Upload**.
6. Open **Serial Monitor** at **115200 baud**.
7. Press the **EN/RST** button on the ESP32. You will see:
   ```
   Connecting to 2.4GHz Wi-Fi SSID: ROVER_NET
   .....
   CONNECTED! Set config.yaml devkit_url to: http://192.168.43.50:80
   ```
8. In your laptop browser, visit `http://192.168.43.50/status`. You will see the live JSON telemetry with ultrasonic distance.

---

## 5. Laptop Software Bring-Up

1. Connect your laptop to the phone's `ROVER_NET` Wi-Fi.
2. In the `rover-project` folder, open `config.yaml` and set your IP addresses:
   ```yaml
   robot:
     devkit_url: "http://192.168.43.50:80"       # ESP32 IP
   camera:
     stream_url: "http://192.168.43.1:8080/video" # Phone IP
   ```
3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Download the YOLO model:
   ```bash
   python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
   mkdir -p models && cp yolov8n.pt models/
   ```
5. Run the preflight self-test:
   ```bash
   python -m tools.verify_setup
   ```
   *(All 6 checks should pass).*
6. Start the mission server:
   ```bash
   python -m rover.main
   ```
7. Open your browser to `http://localhost:8000` to access the Mission Control Dashboard.

---

## 6. Professor Demonstration & Presentation Walkthrough

### Arena Setup
* Set up a rectangle (e.g. 2.0 m × 1.5 m) on the floor using cardboard boxes ($\ge 15\text{ cm}$ tall).
* Place a printout of a person or a toy figure at grid cell `(1, 1)`.
* Place a cardboard box obstacle at grid cell `(2, 0)`.

### Presentation Script for Professor
1. **Explain the Edge Architecture**:
   > *"We implemented a decoupled architecture: the ESP32 microcontroller handles high-frequency deterministic motor control, ultrasonic safety stops, and watchdog timers. High-definition video is streamed over Wi-Fi to our laptop edge server, where YOLOv8 runs real-time deep learning inference and A\* / Boustrophedon path planning."*
2. **Show the Dashboard**:
   > *"On our dashboard at localhost:8000, you can see the real-time occupancy grid, visited coverage vs. observed coverage telemetry, and live camera feed with detection bounding boxes."*
3. **Trigger the Autonomous Mission**:
   > *Click 'Start Mission'. The rover systematically covers the arena. When it spots the person, it pauses, executes a confirmation consensus scan, logs the survivor's coordinates on the map, and finishes its search pattern before returning home."*
4. **Demonstrate Safety Failsafes**:
   > *Put a clipboard or hand 15 cm in front of the rover while moving—show how the HC-SR04 triggers an instant hardware abort, overriding the autonomous command.*
