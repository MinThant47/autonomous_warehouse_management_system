"""MQTT adapter for physical AGV node reports.

Subscribe topic: agv/+/node
Payload formats: a map node string (for example ``Gate_1``), an RFID UID, or
JSON such as ``{"node_id": "Gate_1"}`` / ``{"rfid_id": "A1B2C3D4"}``.
RFID UIDs are resolved through ``rfid_node_map.json``. The ``+`` topic level
identifies the robot.
"""
import json
import logging
import os

from scheduler.scheduler import update_robot_node
from robot_events import publish_robot_state, publish_warehouse_alert
from rfid_node_map import resolve_node_id
from warehouse_tasks import create_inbound_warehouse_task

LOG = logging.getLogger(__name__)


def _robot_id_from_topic(topic):
    parts = topic.split("/")
    return parts[1] if len(parts) == 3 else None


def _node_id_from_payload(payload):
    text = payload.decode("utf-8").strip()
    if not text:
        raise ValueError("Empty MQTT payload")
    try:
        decoded = json.loads(text)
    except json.JSONDecodeError:
        return resolve_node_id(text)
    if not isinstance(decoded, dict):
        raise ValueError("JSON MQTT payload must be an object")
    identifier = decoded.get("rfid_id", decoded.get("rfid", decoded.get("node_id", decoded.get("node"))))
    if not isinstance(identifier, str) or not identifier.strip():
        raise ValueError("JSON payload requires a non-empty node_id or rfid_id")
    return resolve_node_id(identifier)


def _inbound_details_from_payload(payload):
    """Read an ESP32-CAM inbound report with a serial code and pickup node."""
    text = payload.decode("utf-8").strip()
    if not text:
        raise ValueError("Empty MQTT payload")
    try:
        decoded = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError("Inbound MQTT payload must be JSON") from error
    if not isinstance(decoded, dict):
        raise ValueError("Inbound MQTT payload must be a JSON object")

    serial_code = decoded.get("serial_code")
    pickup_location = decoded.get("pickup_location")
    if not isinstance(serial_code, str) or not serial_code.strip():
        raise ValueError("Inbound MQTT payload requires a non-empty serial_code")
    if not isinstance(pickup_location, str) or not pickup_location.strip():
        raise ValueError("Inbound MQTT payload requires a non-empty pickup_location")
    return serial_code.strip(), pickup_location.strip()


def start_mqtt_gateway():
    """Start the background MQTT subscriber, or do nothing when disabled."""
    if os.getenv("MQTT_ENABLED", "false").lower() not in {"1", "true", "yes"}:
        LOG.info("MQTT gateway is disabled (set MQTT_ENABLED=true to enable it).")
        return None

    try:
        import paho.mqtt.client as mqtt
    except ImportError as error:
        raise RuntimeError("Install backend requirements to enable MQTT.") from error

    node_topic = os.getenv("MQTT_NODE_TOPIC", "agv/+/node")
    inbound_topic = os.getenv("MQTT_INBOUND_TOPIC", "cam/inbound")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    username = os.getenv("MQTT_USERNAME")
    if username:
        client.username_pw_set(username, os.getenv("MQTT_PASSWORD"))

    def on_connect(mqtt_client, _userdata, _flags, reason_code, _properties):
        if reason_code.is_failure:
            LOG.error("MQTT connection failed: %s", reason_code)
            return
        mqtt_client.subscribe(node_topic, qos=1)
        mqtt_client.subscribe(inbound_topic, qos=1)
        LOG.info("MQTT connected; subscribed to %s and %s", node_topic, inbound_topic)

    def on_message(_client, _userdata, message):
        try:
            if message.topic == inbound_topic:
                serial_code, pickup_location = _inbound_details_from_payload(message.payload)
                task, result, storage = create_inbound_warehouse_task(serial_code, pickup_location)
                publish_robot_state(result["robot_state"])
                LOG.info(
                    "Created inbound task %s from %s to %s",
                    task["id"], pickup_location, storage["dropoff_location"],
                )
                return

            robot_id = _robot_id_from_topic(message.topic)
            if not robot_id:
                raise ValueError("Expected topic agv/<robot_id>/node")
            state = update_robot_node(robot_id, _node_id_from_payload(message.payload))
            publish_robot_state(state)
            LOG.info("Updated %s at %s", robot_id, state["node"])
        except (UnicodeDecodeError, ValueError) as error:
            LOG.warning("Ignoring MQTT message on %s: %s", message.topic, error)
            if message.topic == inbound_topic:
                publish_warehouse_alert(str(error), locals().get("serial_code"))

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect_async(os.getenv("MQTT_HOST", "localhost"), int(os.getenv("MQTT_PORT", "1883")), 60)
    client.loop_start()
    return client
