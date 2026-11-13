# Firmware Guide — Single ESP32 DevKit Configuration

In this setup, your **Android Smartphone** acts as the high-resolution camera via the **IP Webcam** app.

### What to Flash:
* **Flash ONLY**: `rover_control/rover_control.ino` to your DOIT ESP32 DEVKIT V1 board.
* **Do NOT Flash**: `cam_stream/cam_stream.ino` is for a standalone ESP32-CAM board. Since you are using your mobile phone as the camera, you can ignore this folder entirely.

### Quick Upload Instructions:
1. Open Arduino IDE.
2. Open `firmware/rover_control/rover_control.ino`.
3. Set your Wi-Fi SSID and Password at lines 25–28:
   ```cpp
   #define WIFI_SSID "ROVER_NET"
   #define WIFI_PASS "roverpass123"
   ```
4. Select **DOIT ESP32 DEVKIT V1** under Tools > Board > ESP32 Arduino.
5. Select the COM port and click **Upload**.
6. Open Serial Monitor (115200 baud) to view the assigned IP address.
