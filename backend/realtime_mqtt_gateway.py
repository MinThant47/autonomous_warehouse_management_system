"""MQTT bridge for RFID reports and one-step replanned ESP32 commands.

Topics:
  agv/<robot_id>/node     RFID UID or JSON node report from the ESP32
  agv/<robot_id>/command  one JSON movement command sent back to the ESP32
  agv/<robot_id>/inbound  optional inbound inventory report
"""
import ipaddress
import json
import logging
import math
import os

from rfid_node_map import resolve_node_id
from realtime_navigation import NodeCommandPlanner
from robot_events import publish_camera_url, publish_robot_state, publish_warehouse_alert
from scheduler.scheduler import robots, update_robot_node
from warehouse_tasks import create_inbound_warehouse_task


LOG = logging.getLogger(__name__)
COMMAND_PLANNER = NodeCommandPlanner()


def _robot_id_from_topic(topic):
    parts = topic.split("/")
    return parts[1] if len(parts) == 3 and parts[0] == "agv" else None


def _node_id_from_payload(payload):
    text = payload.decode("utf-8").strip()
    if not text:
        raise ValueError("Empty MQTT payload")
    try:
        decoded = json.loads(text)
    except json.JSONDecodeError:
        return resolve_node_id(text)
    if not isinstance(decoded, dict):
        raise ValueError("JSON node payload must be an object")
    identifier = decoded.get("rfid_id", decoded.get("rfid", decoded.get("node_id", decoded.get("node"))))
    if not isinstance(identifier, str) or not identifier.strip():
        raise ValueError("Node payload requires a non-empty RFID ID or node ID")
    return resolve_node_id(identifier)


def _virtual_state_from_payload(payload, robot_id, current_node, goal):
    """Read optional on-edge localization fields without changing RFID reports."""
    try:
        decoded = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(decoded, dict):
        return None

    required = ("previous_node", "next_node", "current_position", "heading")
    present = [field in decoded for field in required]
    if not any(present):
        return None
    if not all(present):
        raise ValueError("On-edge localization requires previous_node, next_node, current_position, and heading")

    position = decoded["current_position"]
    if not isinstance(position, (list, tuple)) or len(position) != 2:
        raise ValueError("current_position must be a two-number array")
    try:
        x, y = float(position[0]), float(position[1])
        heading = float(decoded["heading"])
        distance_from_previous = float(decoded.get("distance_from_previous_cm", 0.0))
        distance_to_next = float(decoded.get("distance_to_next_cm", 0.0))
    except (TypeError, ValueError) as error:
        raise ValueError("Localization coordinates, heading, and distances must be numbers") from error
    if not all(math.isfinite(value) for value in (x, y, heading, distance_from_previous, distance_to_next)):
        raise ValueError("Localization values must be finite numbers")
    return {
        "robot_id": robot_id,
        "current_node": current_node,
        "previous_node": decoded["previous_node"],
        "next_node": decoded["next_node"],
        "destination": goal,
        "current_position": (x, y),
        "heading": heading,
        "distance_from_previous_cm": distance_from_previous,
        "distance_to_next_cm": distance_to_next,
    }


def _inbound_details_from_payload(payload):
    try:
        decoded = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("Inbound MQTT payload must be JSON") from error
    if not isinstance(decoded, dict):
        raise ValueError("Inbound MQTT payload must be a JSON object")
    serial_code = decoded.get("serial_code")
    pickup_location = decoded.get("pickup_location")
    if not isinstance(serial_code, str) or not serial_code.strip():
        raise ValueError("Inbound MQTT payload requires serial_code")
    if not isinstance(pickup_location, str) or not pickup_location.strip():
        raise ValueError("Inbound MQTT payload requires pickup_location")
    return serial_code.strip(), pickup_location.strip()


def _task_goal(robot_id):
    """Return the active task's next milestone after a node confirmation."""
    current_task = robots[robot_id].get("current_task")
    if not current_task:
        return None
    return current_task["DL"] if current_task["status"] == "TO_DROPOFF" else current_task["PL"]


def start_realtime_mqtt_gateway():
    """Start the RFID-to-command MQTT loop when MQTT_ENABLED is true."""
    if os.getenv("MQTT_ENABLED", "false").lower() not in {"1", "true", "yes"}:
        LOG.info("Realtime MQTT gateway is disabled (set MQTT_ENABLED=true to enable it).")
        return None

    try:
        import paho.mqtt.client as mqtt
    except ImportError as error:
        raise RuntimeError("Install backend requirements to enable MQTT.") from error

    node_topic = os.getenv("MQTT_NODE_TOPIC", "agv/+/node")
    inbound_topic = os.getenv("MQTT_INBOUND_TOPIC", "cam/inbound")
    camera_ip_topic = os.getenv("MQTT_CAMERA_IP_TOPIC", "cam/qrip")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if username := os.getenv("MQTT_USERNAME"):
        client.username_pw_set(username, os.getenv("MQTT_PASSWORD"))

    def on_connect(mqtt_client, _userdata, _flags, reason_code, _properties):
        if reason_code.is_failure:
            LOG.error("MQTT connection failed: %s", reason_code)
            return
        mqtt_client.subscribe(node_topic, qos=1)
        mqtt_client.subscribe(inbound_topic, qos=1)
        mqtt_client.subscribe(camera_ip_topic, qos=1)
        LOG.info("Realtime MQTT connected; subscribed to %s, %s, and %s", node_topic, inbound_topic, camera_ip_topic)

    def on_message(mqtt_client, _userdata, message):
        try:
            if message.topic == camera_ip_topic:
                payload = json.loads(message.payload.decode("utf-8"))
                camera_ip = payload.get("qrip") if isinstance(payload, dict) else None
                if not isinstance(camera_ip, str):
                    raise ValueError("Camera IP message requires a qrip string")
                camera_ip = str(ipaddress.ip_address(camera_ip.strip()))
                publish_camera_url(f"http://{camera_ip}/")
                LOG.info("QR camera is available at http://%s/", camera_ip)
                return

            if message.topic == inbound_topic:
                serial_code, pickup_location = _inbound_details_from_payload(message.payload)
                _task, result, _storage = create_inbound_warehouse_task(serial_code, pickup_location)
                publish_robot_state(result["robot_state"])
                return

            robot_id = _robot_id_from_topic(message.topic)
            if not robot_id or robot_id not in robots:
                raise ValueError("Expected a known robot topic: agv/<robot_id>/node")

            node_id = _node_id_from_payload(message.payload)
            state = update_robot_node(robot_id, node_id, source="mqtt-rfid")
            goal = _task_goal(robot_id)
            virtual_state = _virtual_state_from_payload(message.payload, robot_id, node_id, goal) if goal else None
            command = (
                COMMAND_PLANNER.command_for_virtual_state(virtual_state)
                if virtual_state else COMMAND_PLANNER.command_for_node(robot_id, node_id, goal)
                if goal else {"action": "STOP", "current_node": node_id, "next_node": None, "goal": None, "path": []}
            )
            command_topic = f"agv/{robot_id}/command"
            message_info = mqtt_client.publish(command_topic, json.dumps(command), qos=1)
            if message_info.rc != mqtt.MQTT_ERR_SUCCESS:
                raise ValueError(f"Unable to publish command to {command_topic}")
            publish_robot_state(state)
            LOG.info("%s reached %s; sent %s", robot_id, node_id, command["action"])
        except (UnicodeDecodeError, ValueError) as error:
            LOG.warning("Ignoring MQTT message on %s: %s", message.topic, error)
            if message.topic == inbound_topic:
                publish_warehouse_alert(str(error), locals().get("serial_code"))

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect_async(os.getenv("MQTT_HOST", "localhost"), int(os.getenv("MQTT_PORT", "1883")), 60)
    client.loop_start()
    return client
