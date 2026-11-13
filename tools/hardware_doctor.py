"""Hardware Doctor: tests connectivity to ESP32 DevKit and phone camera stream."""
from __future__ import annotations

import requests
from rover.config import load_calibration, load_config


def main() -> None:
    cfg = load_config("config.yaml")
    cal = load_calibration("calibration.json")
    print("==================================================================")
    print("  AI RECON ROVER v5.6.2 — HARDWARE DIAGNOSTIC DOCTOR")
    print("==================================================================")

    # 1. Test ESP32 Controller (/status)
    devkit_url = cfg.robot.devkit_url.rstrip("/")
    print(f"[1/3] Pinging ESP32 DevKit at: {devkit_url}/status ...")
    esp32_ok = False
    try:
        r = requests.get(
            f"{devkit_url}/status",
            headers={"Connection": "close"},
            timeout=3.0,
        )
        if r.status_code == 200:
            data = r.json()
            esp32_ok = True
            print(
                f"  [PASS] ESP32 Connected! FW: {data.get('fw_version', '5.x')}, "
                f"State: {data.get('state')}, Ultrasonic: {data.get('ultrasonic_cm')} cm, "
                f"RSSI: {data.get('rssi')} dBm"
            )
        else:
            print(f"  [FAIL] ESP32 responded with HTTP {r.status_code}")
    except Exception as exc:
        print(f"  [FAIL] Could not connect to ESP32 at {devkit_url}: {exc}")
        print("         -> Check that ESP32 is powered on, connected to the same Wi-Fi hotspot,")
        print("            and verify its IP in Arduino Serial Monitor (115200 baud) or config.yaml.")

    # 2. Test ESP32 Calibration Sync (/config)
    print(f"\n[2/3] Syncing Calibration Profile to: {devkit_url}/config ...")
    if esp32_ok:
        try:
            r_cfg = requests.post(
                f"{devkit_url}/config",
                json=cal.to_dict(),
                headers={"Connection": "close"},
                timeout=3.0,
            )
            if r_cfg.status_code == 200:
                print(
                    f"  [PASS] Calibration synced! speed_pwm={cal.speed_pwm}, "
                    f"turn_pwm={cal.turn_pwm}, stop_distance_cm={cal.stop_distance_cm}cm"
                )
            else:
                print(f"  [FAIL] ESP32 /config returned HTTP {r_cfg.status_code}")
        except Exception as exc:
            print(f"  [FAIL] Could not push calibration to ESP32: {exc}")
    else:
        print("  [SKIP] Skipped /config sync because ESP32 /status was unreachable.")

    # 3. Test Phone Camera Stream
    print(f"\n[3/3] Checking Phone Camera Stream at: {cfg.camera.stream_url} ...")
    try:
        r_cam = requests.get(
            cfg.camera.stream_url,
            headers={"Connection": "close"},
            stream=True,
            timeout=3.5,
        )
        if r_cam.status_code == 200:
            print("  [PASS] Phone Camera Stream is LIVE and readable!")
        else:
            print(f"  [FAIL] Camera stream responded with HTTP {r_cam.status_code}")
        r_cam.close()
    except Exception as exc:
        print(f"  [FAIL] Could not reach camera stream: {exc}")
        print("         -> Make sure DroidCam / IP Webcam app is open on your phone")
        print("            and stream_url in config.yaml matches the IP shown on the phone screen.")

    print("==================================================================")


if __name__ == "__main__":
    main()
