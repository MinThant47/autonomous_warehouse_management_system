"""Convert encoder tick deltas to approximate wheel travel in centimeters."""
import json
import math
from pathlib import Path


CALIBRATION_FILE = Path(__file__).with_name("encoder_calibration.json")


def calculate_distance_delta_cm(robot_id, delta_ticks_left, delta_ticks_right):
    """Return estimated average wheel travel in cm, or None if factors are invalid.

    The calibration file stores independent ticks-per-centimeter factors for each
    wheel. Absolute deltas are used because the current AGV telemetry contract
    exposes cumulative tick counters, not signed direction.
    """
    try:
        config = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None

    if not isinstance(config, dict) or not isinstance(config.get("robots"), dict):
        return None
    calibration = config["robots"].get(robot_id, {})
    if not isinstance(calibration, dict):
        return None
    ticks_per_cm_left = calibration.get("ticks_per_cm_left")
    ticks_per_cm_right = calibration.get("ticks_per_cm_right")
    factors = (ticks_per_cm_left, ticks_per_cm_right)
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
        for value in factors
    ):
        return None

    left_cm = abs(delta_ticks_left) / ticks_per_cm_left
    right_cm = abs(delta_ticks_right) / ticks_per_cm_right
    return (left_cm + right_cm) / 2


def is_encoder_calibrated(robot_id):
    """Return whether the configured factors were measured for this robot."""
    try:
        config = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(config, dict) or not isinstance(config.get("robots"), dict):
        return False
    calibration = config["robots"].get(robot_id)
    return isinstance(calibration, dict) and calibration.get("calibrated") is True
