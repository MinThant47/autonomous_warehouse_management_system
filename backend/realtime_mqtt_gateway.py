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
import time
from datetime import datetime

from rfid_node_map import resolve_node_id
from realtime_navigation import NodeCommandPlanner, forklift_task_for_route, movement_action_for_vectors
from new_warehouse_map import G, nodes
from virtual_replanning import LiveVirtualReplanner
from idle_return_replanning import replan_interrupted_idle_return
from robot_events import (
    publish_camera_url,
    publish_robot_state,
    publish_traffic_control_state,
    publish_warehouse_alert,
)
from robot_config import TRAFFIC_MAX_ENCODER_AGE_SECONDS, TRAFFIC_REPLAN_TRIGGER_DISTANCE_CM
from scheduler.scheduler import (
    record_robot_command,
    robot_state,
    robots,
    update_robot_encoder_telemetry,
    update_robot_node,
)
from traffic_manager import TrafficManager
from warehouse_tasks import create_inbound_warehouse_task


LOG = logging.getLogger(__name__)
COMMAND_PLANNER = NodeCommandPlanner()
VIRTUAL_REPLANNER = LiveVirtualReplanner()
TRAFFIC_MANAGER = TrafficManager({"R1": "Parking_1", "R2": "Parking_2"})
MQTT_CLIENT = None
_YIELD_IN_FLIGHT = set()
_YIELD_ROUTES = {}


def set_traffic_control_enabled(enabled):
    """Toggle cross-robot reservations and release queued moves when disabling."""
    waiting = TRAFFIC_MANAGER.waiting_commands() if not enabled else []
    TRAFFIC_MANAGER.set_enabled(enabled)
    resumed = []
    errors = []

    for robot_id, command in waiting:
        try:
            topic = publish_robot_command(robot_id, command)
        except RuntimeError as error:
            errors.append({"robot_id": robot_id, "error": str(error)})
            continue
        if topic is None:
            continue
        _apply_command_route(robot_id, command)
        publish_robot_state(robot_state(robot_id))
        resumed.append(robot_id)

    publish_traffic_control_state(TRAFFIC_MANAGER.enabled)
    return {
        "enabled": TRAFFIC_MANAGER.enabled,
        "resumed_robot_ids": resumed,
        "errors": errors,
    }


def publish_robot_command(robot_id, command):
    """Publish an immediate command when the MQTT gateway is connected."""
    if MQTT_CLIENT is None or not MQTT_CLIENT.is_connected():
        raise RuntimeError("MQTT gateway is not connected")
    has_next_node = isinstance(command.get("next_node"), str)
    if has_next_node and not TRAFFIC_MANAGER.reserve_command(robot_id, command):
        robots[robot_id]["waiting_for_traffic"] = True
        publish_robot_state(robot_state(robot_id))
        return None

    import paho.mqtt.client as mqtt

    topic = f"agv/{robot_id}/command"
    result = MQTT_CLIENT.publish(topic, json.dumps(command), qos=1)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"Unable to publish command to {topic}")
    record_robot_command(robot_id, command)
    robots[robot_id]["waiting_for_traffic"] = False
    return topic


def _apply_command_route(robot_id, command):
    """Refresh the displayed edge and route after a movement command is sent."""
    next_node = command.get("next_node")
    current_edge = robots[robot_id].get("map_edge")
    if not (
        current_edge
        and current_edge.get("from_node") == command.get("current_node")
        and current_edge.get("to_node") == next_node
    ):
        robots[robot_id]["map_edge"] = (
            {"from_node": command["current_node"], "to_node": next_node, "distance_origin_cm": 0.0}
            if next_node in nodes
            else None
        )
    path = command.get("path", [])
    remaining_path = path[1:] if path and path[0] == command.get("current_node") else path
    robots[robot_id]["display_route"] = [route_node for route_node in remaining_path if route_node in nodes]


def _retry_waiting_commands(exclude_robot_id=None):
    """Retry queued commands after an RFID report frees a road segment."""
    for robot_id, command in TRAFFIC_MANAGER.waiting_commands():
        if robot_id == exclude_robot_id:
            continue
        try:
            topic = publish_robot_command(robot_id, command)
        except RuntimeError as error:
            LOG.warning("Could not resume %s after a traffic wait: %s", robot_id, error)
            continue
        if topic is None:
            continue
        _apply_command_route(robot_id, command)
        publish_robot_state(robot_state(robot_id))
        LOG.info("Resumed %s after its next segment became available", robot_id)


def _route_edges(robot_id):
    """Return the robot's live edge followed by its planned RFID route."""
    robot = robots[robot_id]
    route = []
    edge = robot.get("map_edge") or {}
    start, end = edge.get("from_node"), edge.get("to_node")
    if start in nodes and end in nodes:
        route.extend((start, end))
    else:
        start = robot.get("node")
        if start in nodes:
            route.append(start)
    for node in robot.get("display_route", []):
        if node in nodes and (not route or node != route[-1]):
            route.append(node)
    return list(zip(route, route[1:]))


def _edge_length_cm(edge):
    from new_warehouse_map import G
    if not G.has_edge(*edge):
        return None
    return float(G.edges[edge].get("weight", 1)) * 2.54


def _telemetry_age_seconds(robot_id):
    telemetry = robots[robot_id].get("encoder_telemetry") or {}
    try:
        return time.time() - datetime.fromisoformat(telemetry.get("received_at", "")).timestamp()
    except (TypeError, ValueError):
        return float("inf")


def _distance_to_route_edge(robot_id, edge):
    """Estimate path distance to an edge from the live encoder/map state."""
    route = _route_edges(robot_id)
    if not route:
        return None
    live = robots[robot_id].get("map_edge") or {}
    telemetry = robots[robot_id].get("encoder_telemetry") or {}
    try:
        received_at = telemetry.get("received_at")
        age = time.time() - datetime.fromisoformat(received_at).timestamp() if received_at else float("inf")
    except (TypeError, ValueError):
        age = float("inf")
    distance = 0.0
    for index, candidate in enumerate(route):
        if candidate == edge:
            if index == 0 and live.get("from_node") == candidate[0] and live.get("to_node") == candidate[1]:
                progress = robots[robot_id].get("distance_since_last_node_cm")
                if age <= TRAFFIC_MAX_ENCODER_AGE_SECONDS and progress is not None:
                    return max(0.0, distance)
            return distance
        if candidate == (edge[1], edge[0]):
            if index == 0 and live.get("from_node") == candidate[0] and live.get("to_node") == candidate[1]:
                progress = robots[robot_id].get("distance_since_last_node_cm")
                length = _edge_length_cm(candidate)
                if age <= TRAFFIC_MAX_ENCODER_AGE_SECONDS and progress is not None and length is not None:
                    return max(0.0, length - float(progress))
            return distance
        length = _edge_length_cm(candidate)
        if length is None:
            return None
        if index == 0 and live.get("from_node") == candidate[0] and live.get("to_node") == candidate[1]:
            progress = robots[robot_id].get("distance_since_last_node_cm")
            if age <= TRAFFIC_MAX_ENCODER_AGE_SECONDS and progress is not None:
                distance += max(0.0, length - float(progress))
            else:
                # Without fresh encoder distance, do not claim a close conflict.
                return None
        else:
            distance += length
    return None


def _resolve_dropoff_conflict():
    """Replan only a lower-priority robot approaching a shared opposing edge."""
    if not TRAFFIC_MANAGER.enabled or MQTT_CLIENT is None or not MQTT_CLIENT.is_connected():
        return False
    stages = {robot_id: _task_goal(robot_id)[1] for robot_id in robots}
    if "DROPOFF" not in stages.values():
        return False
    dropoffs = [robot_id for robot_id, stage in stages.items() if stage == "DROPOFF"]
    for priority_robot in dropoffs:
        for yielding_robot in robots:
            if yielding_robot == priority_robot or stages[yielding_robot] == "DROPOFF":
                continue
            route_a = _route_edges(priority_robot)
            route_b = _route_edges(yielding_robot)
            conflict = next(
                (edge for edge in route_a if (edge[1], edge[0]) in route_b),
                None,
            )
            if conflict is None:
                continue
            if any(
                _telemetry_age_seconds(candidate) > TRAFFIC_MAX_ENCODER_AGE_SECONDS
                or robots[candidate].get("distance_since_last_node_cm") is None
                for candidate in (priority_robot, yielding_robot)
            ):
                continue
            resource_key = (yielding_robot, priority_robot, tuple(sorted(conflict)))
            if resource_key in _YIELD_IN_FLIGHT:
                continue
            a_distance = _distance_to_route_edge(priority_robot, conflict)
            b_distance = _distance_to_route_edge(yielding_robot, (conflict[1], conflict[0]))
            priority_live = robots[priority_robot].get("map_edge") or {}
            yielding_live = robots[yielding_robot].get("map_edge") or {}
            priority_on_edge = (priority_live.get("from_node"), priority_live.get("to_node")) == conflict
            yielding_on_edge = (yielding_live.get("from_node"), yielding_live.get("to_node")) == (conflict[1], conflict[0])
            edge_length = _edge_length_cm(conflict)
            priority_progress = robots[priority_robot].get("distance_since_last_node_cm")
            yielding_progress = robots[yielding_robot].get("distance_since_last_node_cm")
            if edge_length is not None:
                if priority_on_edge and yielding_on_edge and priority_progress is not None and yielding_progress is not None:
                    # Both are already on the same corridor, approaching each other.
                    a_distance = max(0.0, edge_length - float(priority_progress) - float(yielding_progress))
                    b_distance = 0.0
                elif priority_on_edge and priority_progress is not None:
                    # Priority AGV is on the conflict edge; loser is approaching its far endpoint.
                    a_distance = max(0.0, edge_length - float(priority_progress)) + (b_distance or 0.0)
                    b_distance = 0.0
                elif yielding_on_edge and yielding_progress is not None:
                    b_distance = max(0.0, edge_length - float(yielding_progress)) + (a_distance or 0.0)
                    a_distance = 0.0
                elif a_distance is not None and b_distance is not None:
                    # Both are approaching opposite ends of the shared edge.
                    a_distance += edge_length + b_distance
            if a_distance is None or b_distance is None or a_distance + b_distance > TRAFFIC_REPLAN_TRIGGER_DISTANCE_CM:
                continue

            yielding_state = robots[yielding_robot]
            live_edge = yielding_state.get("map_edge") or {}
            if (
                not live_edge.get("from_node") or not live_edge.get("to_node")
                or yielding_state.get("distance_since_last_node_cm") is None
            ):
                LOG.warning("Cannot yield %s yet: fresh encoder position is required", yielding_robot)
                continue
            goal, stage = _task_goal(yielding_robot)
            if not goal:
                continue
            blocked_edge = tuple(sorted(conflict))
            try:
                plan = VIRTUAL_REPLANNER.plan_from_encoder(
                    yielding_robot,
                    live_edge["from_node"],
                    live_edge["to_node"],
                    yielding_state["distance_since_last_node_cm"],
                    goal,
                    heading_degrees=yielding_state.get("heading_degrees"),
                    blocked_edges={blocked_edge},
                    # Do not let a virtual mid-edge start reconnect to the
                    # drop-off side of the contested edge. That would merely
                    # route the yielding AGV through the conflict before its
                    # graph search could honor the removed edge.
                    blocked_nodes={conflict[0]},
                )
            except (ValueError, KeyError) as error:
                LOG.warning("Could not replan yielding robot %s: %s", yielding_robot, error)
                continue
            route = plan.get("path") or []
            next_node = plan.get("first_reentry_node")
            if not plan.get("success") or not next_node or next_node not in nodes:
                LOG.warning("No safe detour found for %s; leaving it stopped by traffic reservation", yielding_robot)
                continue
            previous = yielding_state.get("previous_node")
            command = {
                "action": plan.get("first_action") or "STOP",
                "task": stage,
                "forklift_task": "NONE",
                "current_node": live_edge["from_node"],
                "next_node": next_node,
                "goal": goal,
                "path": route,
                "previous_node": previous,
            }
            try:
                topic = publish_robot_command(yielding_robot, command)
            except RuntimeError as error:
                LOG.warning("Could not publish yield command for %s: %s", yielding_robot, error)
                continue
            if topic is None:
                continue
            _YIELD_IN_FLIGHT.add(resource_key)
            _YIELD_ROUTES[yielding_robot] = {
                "priority_robot": priority_robot,
                "priority_edge": conflict,
                "blocked_node": conflict[0],
                "goal": goal,
                "task_stage": stage,
                "remaining_nodes": [node for node in route[1:] if node in nodes],
            }
            _apply_command_route(yielding_robot, command)
            # Keep the map marker at the encoder-estimated mid-edge point while
            # this tactical command moves to its first RFID re-entry node.
            yielding_state["map_edge"] = {
                "from_node": live_edge["from_node"],
                "to_node": next_node,
                "start_position": [
                    coordinate / 2.54 for coordinate in plan["state"]["current_position_cm"]
                ],
                "end_position": list(nodes[next_node]),
                "distance_origin_cm": float(yielding_state["distance_since_last_node_cm"]),
            }
            publish_robot_state(robot_state(yielding_robot))
            LOG.info(
                "%s yielded to %s near opposing edge %s by replanning around it (%s)",
                yielding_robot, priority_robot, blocked_edge, route,
            )
            return True
    return False


def _continue_yield_route(robot_id, current_node):
    """Continue a tactical detour at RFID nodes while the drop-off conflict remains."""
    plan = _YIELD_ROUTES.get(robot_id)
    if not plan:
        return None
    goal, stage = _task_goal(robot_id)
    priority_route = _route_edges(plan["priority_robot"])
    conflict_active = plan["priority_edge"] in priority_route
    if goal != plan["goal"] or stage != plan["task_stage"] or not conflict_active:
        _YIELD_ROUTES.pop(robot_id, None)
        _YIELD_IN_FLIGHT.difference_update(
            key for key in tuple(_YIELD_IN_FLIGHT) if key[0] == robot_id
        )
        return None

    remaining = plan["remaining_nodes"]
    if remaining and remaining[0] == current_node:
        remaining.pop(0)
    elif remaining:
        # An unexpected RFID report invalidates the cached tactical route.
        _YIELD_ROUTES.pop(robot_id, None)
        _YIELD_IN_FLIGHT.difference_update(
            key for key in tuple(_YIELD_IN_FLIGHT) if key[0] == robot_id
        )
        return None
    if not remaining:
        _YIELD_ROUTES.pop(robot_id, None)
        _YIELD_IN_FLIGHT.difference_update(
            key for key in tuple(_YIELD_IN_FLIGHT) if key[0] == robot_id
        )
        return None

    route = [current_node, *remaining]
    next_node = remaining[0]
    heading = robots[robot_id].get("heading_vector")
    movement = COMMAND_PLANNER._unit_vector(current_node, next_node)
    action = movement_action_for_vectors(
        heading or movement,
        movement,
        at_intersection=G.degree[current_node] >= 3,
    )
    edge = COMMAND_PLANNER.current_edge_for_robot(robot_id) or {}
    return {
        "action": action,
        "task": stage,
        "forklift_task": forklift_task_for_route(stage, current_node, route),
        "current_node": current_node,
        "next_node": next_node,
        "goal": goal,
        "path": route,
        "previous_node": edge.get("previous_node"),
    }


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
                update_robot_encoder_telemetry(robot_id, ticks_left, ticks_right)
                _resolve_dropoff_conflict()
                publish_robot_state(robot_state(robot_id))
                LOG.debug("Stored raw encoder telemetry for %s: left=%s right=%s", robot_id, ticks_left, ticks_right)
                return

            robot_id = _robot_id_from_topic(message.topic)
            if not robot_id or robot_id not in robots:
                raise ValueError("Expected a known robot topic: agv/<robot_id>/node")

            node_id = _node_id_from_payload(message.payload)
            state = update_robot_node(robot_id, node_id, source="mqtt-rfid")
            if not TRAFFIC_MANAGER.observe_node(robot_id, node_id):
                robots[robot_id]["waiting_for_traffic"] = True
                blocked_command = {
                    "action": "STOP",
                    "task": "NONE",
                    "forklift_task": "NONE",
                    "current_node": node_id,
                    "next_node": None,
                    "goal": None,
                    "path": [],
                    "previous_node": None,
                }
                publish_robot_command(robot_id, blocked_command)
                robots[robot_id]["waiting_for_traffic"] = True
                publish_robot_state(robot_state(robot_id))
                LOG.error("%s reported occupied node %s; holding for operator review", robot_id, node_id)
                return

            yield_command = _continue_yield_route(robot_id, node_id)
            if yield_command:
                try:
                    yield_topic = publish_robot_command(robot_id, yield_command)
                except RuntimeError as error:
                    raise ValueError(str(error)) from error
                if yield_topic is not None:
                    _apply_command_route(robot_id, yield_command)
                publish_robot_state(robot_state(robot_id))
                LOG.info("%s continues its traffic yield route from %s", robot_id, node_id)
                return

            _retry_waiting_commands(exclude_robot_id=robot_id)
            goal, task_stage = _task_goal(robot_id)
            command = (
                COMMAND_PLANNER.command_for_node(
                    robot_id, node_id, goal, task_stage,
                    heading_vector=robots[robot_id].get("heading_vector"),
                )
                if goal else {
                    "action": "STOP",
                    "task": task_stage,
                    "forklift_task": "NONE",
                    "current_node": node_id,
                    "next_node": None,
                    "goal": None,
                    "path": [],
                    "previous_node": None,
                }
            )
            try:
                command_topic = publish_robot_command(robot_id, command)
            except RuntimeError as error:
                raise ValueError(str(error)) from error
            if command_topic is None:
                robots[robot_id]["map_edge"] = None
                path = command.get("path", [])
                remaining_path = path[1:] if path and path[0] == node_id else path
                robots[robot_id]["display_route"] = [route_node for route_node in remaining_path if route_node in nodes]
                LOG.info("%s is waiting at %s for its next segment reservation", robot_id, node_id)
            else:
                _apply_command_route(robot_id, command)
            _resolve_dropoff_conflict()
            state = robot_state(robot_id)
            publish_robot_state(state)
            if command_topic is not None:
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
