"""MQTT bridge for RFID reports and one-step replanned ESP32 commands.

Topics:
  agv/<robot_id>/node     RFID UID or JSON node report from the ESP32
  agv/<robot_id>/telemetry  JSON telemetry containing raw encoder counters
  agv/<robot_id>/command  one JSON movement command sent back to the ESP32
  agv/<robot_id>/inbound  optional inbound inventory report
"""
import ipaddress
import json
import logging
import os

from rfid_node_map import resolve_node_id
from realtime_navigation import NodeCommandPlanner
from new_warehouse_map import nodes
from virtual_replanning import LiveVirtualReplanner
from idle_return_replanning import replan_interrupted_idle_return
from robot_events import publish_camera_url, publish_robot_state, publish_warehouse_alert
from scheduler.scheduler import robot_state, robots, update_robot_encoder_telemetry, update_robot_node
from warehouse_tasks import create_inbound_warehouse_task


LOG = logging.getLogger(__name__)
COMMAND_PLANNER = NodeCommandPlanner()
VIRTUAL_REPLANNER = LiveVirtualReplanner()
MQTT_CLIENT = None


def publish_robot_command(robot_id, command):
    """Publish an immediate command when the MQTT gateway is connected."""
    if MQTT_CLIENT is None or not MQTT_CLIENT.is_connected():
        raise RuntimeError("MQTT gateway is not connected")
    import paho.mqtt.client as mqtt

    topic = f"agv/{robot_id}/command"
    result = MQTT_CLIENT.publish(topic, json.dumps(command), qos=1)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"Unable to publish command to {topic}")
    return topic


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


def _encoder_ticks_from_payload(payload):
    try:
        decoded = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("Telemetry payload must be JSON") from error
    if not isinstance(decoded, dict):
        raise ValueError("Telemetry payload must be a JSON object")
    ticks_left = decoded.get("ticks_L")
    ticks_right = decoded.get("ticks_R")
    if isinstance(ticks_left, bool) or not isinstance(ticks_left, int):
        raise ValueError("Telemetry payload requires integer ticks_L")
    if isinstance(ticks_right, bool) or not isinstance(ticks_right, int):
        raise ValueError("Telemetry payload requires integer ticks_R")
    return ticks_left, ticks_right


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
    """Return the active task's goal and the forklift stage for the AGV."""
    current_task = robots[robot_id].get("current_task")
    if not current_task:
        idle_goal = robots[robot_id].get("idle_goal")
        if idle_goal:
            return idle_goal, "IDLE"
        return None, "NONE"
    if current_task["status"] == "TO_DROPOFF":
        return current_task["DL"], "DROPOFF"
    return current_task["PL"], "PICKUP"


def start_realtime_mqtt_gateway():
    """Start the RFID-to-command MQTT loop when MQTT_ENABLED is true."""
    global MQTT_CLIENT
    if os.getenv("MQTT_ENABLED", "false").lower() not in {"1", "true", "yes"}:
        LOG.info("Realtime MQTT gateway is disabled (set MQTT_ENABLED=true to enable it).")
        return None

    try:
        import paho.mqtt.client as mqtt
    except ImportError as error:
        raise RuntimeError("Install backend requirements to enable MQTT.") from error

    node_topic = os.getenv("MQTT_NODE_TOPIC", "agv/+/node")
    telemetry_topic = os.getenv("MQTT_TELEMETRY_TOPIC", "agv/+/telemetry")
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
        mqtt_client.subscribe(telemetry_topic, qos=1)
        mqtt_client.subscribe(inbound_topic, qos=1)
        mqtt_client.subscribe(camera_ip_topic, qos=1)
        LOG.info("Realtime MQTT connected; subscribed to %s, %s, %s, and %s", node_topic, telemetry_topic, inbound_topic, camera_ip_topic)

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
                result["idle_return_replan"] = replan_interrupted_idle_return(
                    result, COMMAND_PLANNER, VIRTUAL_REPLANNER, publish_robot_command
                )
                publish_robot_state(result["robot_state"])
                return

            topic_parts = message.topic.split("/")
            if len(topic_parts) == 3 and topic_parts[0] == "agv" and topic_parts[2] == "telemetry":
                robot_id = _robot_id_from_topic(message.topic)
                if not robot_id or robot_id not in robots:
                    raise ValueError("Expected a known telemetry topic: agv/<robot_id>/telemetry")
                ticks_left, ticks_right = _encoder_ticks_from_payload(message.payload)
                state = update_robot_encoder_telemetry(robot_id, ticks_left, ticks_right)
                publish_robot_state(state)
                LOG.debug("Stored raw encoder telemetry for %s: left=%s right=%s", robot_id, ticks_left, ticks_right)
                return

            robot_id = _robot_id_from_topic(message.topic)
            if not robot_id or robot_id not in robots:
                raise ValueError("Expected a known robot topic: agv/<robot_id>/node")

            node_id = _node_id_from_payload(message.payload)
            state = update_robot_node(robot_id, node_id, source="mqtt-rfid")
            goal, task_stage = _task_goal(robot_id)
            command = (
                COMMAND_PLANNER.command_for_node(robot_id, node_id, goal, task_stage)
                if goal else {
                    "action": "STOP",
                    "task": task_stage,
                    "current_node": node_id,
                    "next_node": None,
                    "goal": None,
                    "path": [],
                    "previous_node": None,
                }
            )
            next_node = command.get("next_node")
            command_topic = f"agv/{robot_id}/command"
            message_info = mqtt_client.publish(command_topic, json.dumps(command), qos=1)
            if message_info.rc != mqtt.MQTT_ERR_SUCCESS:
                raise ValueError(f"Unable to publish command to {command_topic}")
            robots[robot_id]["map_edge"] = (
                {"from_node": node_id, "to_node": next_node, "distance_origin_cm": 0.0}
                if next_node in nodes
                else None
            )
            command_path = command.get("path", [])
            remaining_path = command_path[1:] if command_path and command_path[0] == node_id else command_path
            robots[robot_id]["display_route"] = [route_node for route_node in remaining_path if route_node in nodes]
            state = robot_state(robot_id)
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
    MQTT_CLIENT = client
    return client
