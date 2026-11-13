"""Calibration routines for straight speed, asymmetric left/right turn rate, and motor trim."""
from __future__ import annotations

import time
from typing import Any, Dict
from rover.config import CalibrationData
from rover.robot import RobotInterface


def apply_straight_calibration(
    cal: CalibrationData,
    measured_distance_cm: float,
    test_duration_s: float = 2.0,
    lateral_drift_cm: float = 0.0,
) -> None:
    if test_duration_s > 0 and measured_distance_cm > 3.0:
        cal.cm_per_s = round(measured_distance_cm / test_duration_s, 2)
    if abs(lateral_drift_cm) >= 1.5:
        drift_factor = lateral_drift_cm / max(10.0, measured_distance_cm)
        # Positive lateral_drift_cm = drifted Right (left wheel too fast -> reduce left_trim)
        # Negative lateral_drift_cm = drifted Left (right wheel too fast -> reduce right_trim)
        if lateral_drift_cm > 0:
            cal.left_trim = round(max(0.65, min(1.15, cal.left_trim * (1.0 - 0.55 * abs(drift_factor)))), 3)
        else:
            cal.right_trim = round(max(0.65, min(1.15, cal.right_trim * (1.0 - 0.55 * abs(drift_factor)))), 3)
    cal.calibrated_at = time.time()


def apply_turn_calibration(
    cal: CalibrationData,
    actual_angle_deg: float,
    commanded_angle_deg: float = 90.0,
    stop_distance_cm: float = 20.0,
    direction: str = "both",
) -> None:
    cmd_abs = abs(commanded_angle_deg)
    act_abs = abs(actual_angle_deg)
    if cmd_abs > 0 and act_abs > 10.0:
        scale = act_abs / cmd_abs
        d_lower = direction.strip().lower()
        if d_lower == "left":
            base_l = cal.deg_per_s_left if cal.deg_per_s_left > 1.0 else cal.deg_per_s
            cal.deg_per_s_left = round(base_l * scale, 2)
            if cal.deg_per_s_right <= 1.0:
                cal.deg_per_s_right = cal.deg_per_s
            cal.deg_per_s = round(0.5 * (cal.deg_per_s_left + cal.deg_per_s_right), 2)
        elif d_lower == "right":
            base_r = cal.deg_per_s_right if cal.deg_per_s_right > 1.0 else cal.deg_per_s
            cal.deg_per_s_right = round(base_r * scale, 2)
            if cal.deg_per_s_left <= 1.0:
                cal.deg_per_s_left = cal.deg_per_s
            cal.deg_per_s = round(0.5 * (cal.deg_per_s_left + cal.deg_per_s_right), 2)
        else:
            cal.deg_per_s = round(cal.deg_per_s * scale, 2)
            if cal.deg_per_s_left > 1.0:
                cal.deg_per_s_left = round(cal.deg_per_s_left * scale, 2)
            if cal.deg_per_s_right > 1.0:
                cal.deg_per_s_right = round(cal.deg_per_s_right * scale, 2)
    cal.stop_distance_cm = stop_distance_cm
    cal.calibrated_at = time.time()


def run_calibration_test_step(
    robot: RobotInterface,
    cal: CalibrationData,
    step: str,
    duration_or_value: float = 3.0,
) -> Dict[str, Any]:
    if step == "ping":
        st = robot.get_status()
        return {"status": "ok", "robot": st.to_dict()}
    if step == "straight_test":
        dist_cm = cal.cm_per_s * duration_or_value
        st = robot.execute_move("forward", dist_cm)
        return {"status": "ok", "move": "forward", "commanded_cm": dist_cm, "result": st.to_dict()}
    if step == "turn_test":
        st = robot.execute_move("turn", duration_or_value)
        return {"status": "ok", "move": "turn", "commanded_deg": duration_or_value, "result": st.to_dict()}
    if step == "ultrasonic_test":
        st = robot.get_status()
        return {"status": "ok", "ultrasonic_cm": st.ultrasonic_cm}
    return {"status": "error", "message": f"Unknown calibration step: {step}"}
