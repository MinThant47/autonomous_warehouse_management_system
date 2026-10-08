"""Run two map-aware virtual AGVs against the warehouse backend over MQTT."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import paho.mqtt.client as mqtt


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parent
BACKEND_DIRECTORY = REPOSITORY_ROOT / "backend"
sys.path.insert(0, str(BACKEND_DIRECTORY))

try:
    from new_warehouse_map import G, nodes
except ImportError as error:
    raise SystemExit(
        "Could not load the warehouse map. Install this folder's requirements.txt "
        "and keep 'AGV simulate' beside the backend folder."
    ) from error


LOG = logging.getLogger("agv_simulator")
COMMAND_TOPIC = "agv/+/command"
CM_PER_MAP_UNIT = 2.54


@dataclass
class SimulatedRobot:
    robot_id: str
    anchor_node: str
    position: tuple[float, float]
    ticks_per_cm_left: float
    ticks_per_cm_right: float
    ticks_left: int = 0
    ticks_right: int = 0
    fractional_ticks_left: float = 0.0
    fractional_ticks_right: float = 0.0
    target_node: str | None = None
    command: dict = field(default_factory=dict)
    last_command_fingerprint: str | None = None
    last_telemetry_at: float = 0.0
    last_initial_report_at: float = 0.0
    waiting_for_first_command: bool = True

    @property
    def moving(self):
        return self.target_node is not None


class AGVSimulator:
    """Simulate RFID reports, cumulative encoder ticks, and MQTT commands."""

    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.config = self._read_config(config_path)
        simulation = self.config["simulation"]
        self.speed_mps = float(simulation["speed_mps"])
        self.telemetry_interval = float(simulation["telemetry_interval_seconds"])
        self.initial_report_retry = float(simulation["initial_node_retry_seconds"])
        self.map_unit_meters = float(simulation["map_unit_meters"])
        self.maximum_step_seconds = float(simulation["maximum_step_seconds"])
        if self.speed_mps <= 0 or self.telemetry_interval <= 0 or self.map_unit_meters <= 0:
            raise ValueError("Simulation speed, telemetry interval, and map scale must be positive")

        self.robots = {}
        for robot_id, settings in self.config["robots"].items():
            start_node = settings["start_node"]
            if start_node not in nodes:
                raise ValueError(f"Unknown start node for {robot_id}: {start_node}")
            left_factor = float(settings["ticks_per_cm_left"])
            right_factor = float(settings["ticks_per_cm_right"])
            if left_factor <= 0 or right_factor <= 0:
                raise ValueError(f"Encoder tick factors for {robot_id} must be positive")
            self.robots[robot_id] = SimulatedRobot(
                robot_id=robot_id,
                anchor_node=start_node,
                position=tuple(map(float, nodes[start_node])),
                ticks_per_cm_left=left_factor,
                ticks_per_cm_right=right_factor,
            )
        if set(self.robots) != {"R1", "R2"}:
            raise ValueError("The simulator config must define exactly R1 and R2")

        mqtt_config = self.config["mqtt"]
        self.host = os.getenv("MQTT_HOST", mqtt_config.get("host", "localhost"))
        self.port = int(os.getenv("MQTT_PORT", mqtt_config.get("port", 1883)))
        self.keepalive = int(mqtt_config.get("keepalive_seconds", 60))
        self.qos = int(mqtt_config.get("qos", 1))
        self.lock = threading.RLock()
        self.connected = threading.Event()
        self.subscribed = False
        self.first_startup_reports_sent = False
        self.subscription_mid = None
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="warehouse-agv-simulator",
        )
        if username := os.getenv("MQTT_USERNAME"):
            self.client.username_pw_set(username, os.getenv("MQTT_PASSWORD"))
        self.client.on_connect = self._on_connect
        self.client.on_subscribe = self._on_subscribe
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    @staticmethod
    def _read_config(path: Path):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise SystemExit(f"Could not read simulator config {path}: {error}") from error
        if not isinstance(data, dict) or not all(
            isinstance(data.get(key), dict) for key in ("mqtt", "simulation", "robots")
        ):
            raise ValueError("Simulator config requires mqtt, simulation, and robots objects")
        return data

    def _on_connect(self, client, _userdata, _connect_flags, reason_code, _properties):
        if reason_code.is_failure:
            LOG.error("MQTT connection failed: %s", reason_code)
            return
        LOG.info("Connected to MQTT broker %s:%s", self.host, self.port)
        result, mid = client.subscribe(COMMAND_TOPIC, qos=self.qos)
        if result != mqtt.MQTT_ERR_SUCCESS:
            LOG.error("Could not subscribe to %s (rc=%s)", COMMAND_TOPIC, result)
            return
        self.subscription_mid = mid

    def _on_subscribe(self, _client, _userdata, mid, reason_codes, _properties):
        if mid != self.subscription_mid:
            return
        if any(getattr(code, "is_failure", False) for code in reason_codes):
            LOG.error("Broker rejected subscription to %s: %s", COMMAND_TOPIC, reason_codes)
            return
        with self.lock:
            self.subscribed = True
            self.connected.set()
            if not self.first_startup_reports_sent:
                # The backend's configured initial poses are Parking_1/Parking_2.
                # Reporting them after subscribing starts the ordinary RFID loop.
                for robot in self.robots.values():
                    self._publish_node(robot, robot.anchor_node)
                    self._publish_telemetry(robot)
                    robot.last_initial_report_at = time.monotonic()
                self.first_startup_reports_sent = True
        LOG.info("Subscribed to %s; both initial robot positions were reported", COMMAND_TOPIC)

    def _on_disconnect(self, _client, _userdata, _disconnect_flags, reason_code, _properties):
        self.connected.clear()
        self.subscribed = False
        if reason_code.is_failure:
            LOG.warning("MQTT connection lost: %s; waiting for automatic reconnect", reason_code)

    def _on_message(self, _client, _userdata, message):
        parts = message.topic.split("/")
        if len(parts) != 3 or parts[0] != "agv" or parts[2] != "command":
            return
        robot_id = parts[1]
        with self.lock:
            robot = self.robots.get(robot_id)
            if robot is None:
                LOG.warning("Ignoring command for unconfigured robot %s", robot_id)
                return
            try:
                command = json.loads(message.payload.decode("utf-8"))
                self._accept_command(robot, command)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as error:
                LOG.warning("Ignoring invalid command on %s: %s", message.topic, error)

    def _accept_command(self, robot: SimulatedRobot, command):
        if not isinstance(command, dict):
            raise ValueError("command must be a JSON object")
        action = command.get("action")
        source = command.get("current_node")
        target = command.get("next_node")
        if not isinstance(action, str) or not isinstance(source, str):
            raise ValueError("command requires string action and current_node fields")
        if source != robot.anchor_node:
            raise ValueError(
                f"backend current_node={source} does not match simulator RFID anchor={robot.anchor_node}"
            )

        fingerprint = json.dumps(command, sort_keys=True, separators=(",", ":"))
        if fingerprint == robot.last_command_fingerprint:
            LOG.debug("Ignoring duplicate command for %s", robot.robot_id)
            return
        if target is None or action.upper() == "STOP":
            robot.last_command_fingerprint = fingerprint
            robot.waiting_for_first_command = False
            robot.command = command
            robot.target_node = None
            LOG.info("%s received STOP at (or between) RFID nodes", robot.robot_id)
            return
        if not isinstance(target, str) or target not in nodes:
            raise ValueError("next_node must be a known map node or null")
        if target != source and not G.has_edge(source, target):
            raise ValueError(f"Backend requested non-adjacent move {source} -> {target}")

        robot.last_command_fingerprint = fingerprint
        robot.waiting_for_first_command = False
        robot.command = command
        # Actions are retained for the console output. Map motion follows the
        # backend's next_node, including a same-node target used to retreat to
        # the last RFID tag from a point between tags.
        robot.target_node = target
        LOG.info(
            "%s command: %s (%s -> %s), task=%s forklift_task=%s",
            robot.robot_id,
            action.upper(),
            source,
            target,
            command.get("task", "NONE"),
            command.get("forklift_task", "NONE"),
        )

    def _publish_node(self, robot: SimulatedRobot, node_id: str):
        self._publish(f"agv/{robot.robot_id}/node", {"node_id": node_id})
        LOG.info("%s RFID report: %s", robot.robot_id, node_id)

    def _publish_telemetry(self, robot: SimulatedRobot):
        self._publish(
            f"agv/{robot.robot_id}/telemetry",
            {"ticks_L": robot.ticks_left, "ticks_R": robot.ticks_right},
        )

    def _publish(self, topic: str, payload: dict):
        if not self.connected.is_set():
            return
        result = self.client.publish(
            topic,
            json.dumps(payload, separators=(",", ":")),
            qos=self.qos,
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            LOG.warning("MQTT publish failed on %s (rc=%s)", topic, result.rc)

    def _advance_robot(self, robot: SimulatedRobot, elapsed_seconds: float):
        if not robot.moving:
            return 0.0, False
        target_position = tuple(map(float, nodes[robot.target_node]))
        x, y = robot.position
        target_x, target_y = target_position
        delta_x, delta_y = target_x - x, target_y - y
        remaining_units = abs(delta_x) + abs(delta_y)
        if remaining_units <= 1e-9:
            arrived_node = robot.target_node
            robot.anchor_node = arrived_node
            robot.position = target_position
            robot.target_node = None
            robot.command = {}
            return 0.0, True

        if abs(delta_x) > 1e-6 and abs(delta_y) > 1e-6:
            raise ValueError(f"Map edge to {robot.target_node} is not axis-aligned")

        step_meters = min(self.speed_mps * elapsed_seconds, remaining_units * self.map_unit_meters)
        step_units = step_meters / self.map_unit_meters
        ratio = min(1.0, step_units / remaining_units)
        new_position = (x + delta_x * ratio, y + delta_y * ratio)
        travelled_cm = step_meters * 100.0
        left_ticks = robot.fractional_ticks_left + travelled_cm * robot.ticks_per_cm_left
        right_ticks = robot.fractional_ticks_right + travelled_cm * robot.ticks_per_cm_right
        whole_left, whole_right = math.floor(left_ticks), math.floor(right_ticks)
        robot.ticks_left += whole_left
        robot.ticks_right += whole_right
        robot.fractional_ticks_left = left_ticks - whole_left
        robot.fractional_ticks_right = right_ticks - whole_right
        robot.position = new_position

        arrived = ratio >= 1.0 - 1e-9
        if arrived:
            arrived_node = robot.target_node
            robot.anchor_node = arrived_node
            robot.position = target_position
            robot.target_node = None
            robot.command = {}
            return travelled_cm, True
        return travelled_cm, False

    def run(self):
        LOG.info("Connecting simulator to %s:%s", self.host, self.port)
        self.client.connect_async(self.host, self.port, self.keepalive)
        self.client.loop_start()
        last_loop = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                elapsed = min(now - last_loop, self.maximum_step_seconds)
                last_loop = now
                with self.lock:
                    if self.connected.is_set():
                        for robot in self.robots.values():
                            _distance_cm, arrived = self._advance_robot(robot, elapsed)
                            if arrived:
                                self._publish_node(robot, robot.anchor_node)
                            if now - robot.last_telemetry_at >= self.telemetry_interval:
                                self._publish_telemetry(robot)
                                robot.last_telemetry_at = now
                            if (
                                robot.waiting_for_first_command
                                and not robot.moving
                                and now - robot.last_initial_report_at >= self.initial_report_retry
                            ):
                                # Retry only startup RFID reports until each AGV receives
                                # its first backend command; this recovers from startup-order races.
                                self._publish_node(robot, robot.anchor_node)
                                robot.last_initial_report_at = now
                time.sleep(min(self.telemetry_interval / 4, 0.05))
        except KeyboardInterrupt:
            LOG.info("Stopping AGV simulator")
        finally:
            self.connected.clear()
            self.client.loop_stop()
            self.client.disconnect()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=HERE / "config.json",
        help="Path to simulator configuration JSON",
    )
    parser.add_argument("--host", help="Override MQTT broker host (also accepts MQTT_HOST)")
    parser.add_argument("--port", type=int, help="Override MQTT broker port (also accepts MQTT_PORT)")
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=os.getenv("SIM_LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    args = parse_args()
    simulator = AGVSimulator(args.config)
    if args.host:
        simulator.host = args.host
    if args.port:
        simulator.port = args.port
    simulator.run()


if __name__ == "__main__":
    main()
