"""Shared per-robot startup and idle parking configuration."""

ROBOT_HOME_NODES = {
    "R1": "Parking_1",
    "R2": "Parking_2",
}

# Initial simulation setting. Measure real robot speed, command latency, and
# encoder error before relying on this value for physical collision avoidance.
TRAFFIC_REPLAN_TRIGGER_DISTANCE_CM = 100.0
TRAFFIC_MAX_ENCODER_AGE_SECONDS = 2.0
