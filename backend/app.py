import json
import os
from queue import Empty

# macOS can load OpenMP through both the ML scheduler dependencies and
# OpenCV/ONNX dependencies.  Set this before importing either stack so the
# server does not abort during startup.  This is a compatibility workaround;
# the production fix is keeping all ML packages linked to one OpenMP runtime.
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

from flask import Flask, Response, request, jsonify, stream_with_context
from flask_cors import CORS
from waitress import serve
from object_detection.object_detection_app import object_detection_bp
from scheduler.scheduler import cancel_current_task, dispatch_task, robot_state, robots, update_robot_node
from new_warehouse_map import edges, nodes
from realtime_mqtt_gateway import COMMAND_PLANNER, publish_robot_command, start_realtime_mqtt_gateway
from encoder_distance import is_encoder_calibrated
from virtual_replanning import CM_PER_MAP_UNIT, LiveVirtualReplanner
from idle_return_replanning import replan_interrupted_idle_return
from robot_events import get_camera_url, publish_robot_state, publish_warehouse_alert, subscribe, unsubscribe
from warehouse_tasks import create_inbound_warehouse_task, tasks
from database.database import (
    WarehouseDatabaseError,
    create_log,
    get_item_at_shelf,
    get_inventory_items,
    get_item_location,
    get_shelves,
    get_warehouse_monitor,
    move_inventory_item,
    remove_inventory_item,
    rollback_scheduled_task,
)
from database.seed import seed_shelves

app = Flask(__name__)

CORS(app)

# The camera UI and its API now run in this same Flask application and are
# served by the same Waitress process as the warehouse API.
app.register_blueprint(object_detection_bp, url_prefix="/object-detection")
VIRTUAL_REPLANNER = LiveVirtualReplanner()

# Creates the local database and its storage locations.  The database module
# remains independent from Flask, scheduler, MQTT, and robot state.
seed_shelves()

def _map_shelf_id(shelf_id):
    """Convert a database shelf ID (yellow_3) to its map node (Yellow_3)."""
    if not isinstance(shelf_id, str) or "_" not in shelf_id:
        raise WarehouseDatabaseError(f"Invalid shelf ID '{shelf_id}'")
    category, number = shelf_id.split("_", maxsplit=1)
    map_shelf_id = f"{category.capitalize()}_{number}"
    if map_shelf_id not in nodes:
        raise WarehouseDatabaseError(f"Shelf '{shelf_id}' is not a map storage location")
    return map_shelf_id


def _dispatch_warehouse_task(serial_number, task_type, pickup_location, dropoff_location):
    task = {
        "id": len(tasks) + 1,
        "type": task_type,
        "PL": pickup_location,
        "DL": dropoff_location,
        "pickup_time": 3,
        "dropoff_time": 3,
        "deadline": 300,
        "serial_number": serial_number,
    }
    result = dispatch_task(task)
    result["idle_return_replan"] = replan_interrupted_idle_return(
        result, COMMAND_PLANNER, VIRTUAL_REPLANNER, publish_robot_command
    )
    publish_robot_state(result["robot_state"])
    tasks.append(task)
    return task, result

@app.route("/")
def home():
    return {
        "status": "running",
        "message": "Warehouse backend is active"
    }

# @app.route("/task", methods=["POST"])
# def create_task():

#     data = request.json
#     serial_number = data["serial_number"]
#     print(serial_number)

#     task, result = _dispatch_warehouse_task(
#         serial_number, 1, data["pickup_location"], data["dropoff_location"]
#     )

#     print(result)

#     return jsonify({
#         "message": "Task assigned successfully",
#         "task": task,
#         "result": result
#     })


@app.route("/tasks/inbound", methods=["POST"])
def create_inbound_task():
    """Store an arriving catalog item in the first free matching shelf."""
    data = request.get_json(silent=True) or {}
    try:
        task, result, storage = create_inbound_warehouse_task(
            data.get("serial_code"), data.get("pickup_location")
        )
        result["idle_return_replan"] = replan_interrupted_idle_return(
            result, COMMAND_PLANNER, VIRTUAL_REPLANNER, publish_robot_command
        )
        publish_robot_state(result["robot_state"])
    except WarehouseDatabaseError as error:
        publish_warehouse_alert(str(error), data.get("serial_code"))
        return jsonify({"error": str(error)}), 400

    return jsonify({
        "message": "Inbound task assigned successfully",
        "task": task,
        "result": result,
        "category": storage["category"],
        "dropoff_shelf": storage["shelf_id"],
    })


@app.route("/tasks/outbound", methods=["POST"])
def create_outbound_task():
    """Dispatch an item from its recorded shelf to a selected outbound gate."""
    data = request.get_json(silent=True) or {}
    serial_number = data.get("serial_number", "").strip()
    gate = data.get("gate", "")
    if not serial_number or gate not in {"Gate_3", "Gate_4"}:
        return jsonify({"error": "serial_number and Gate_3 or Gate_4 are required"}), 400

    try:
        location = get_item_location(serial_number)
        if not location:
            raise WarehouseDatabaseError(f"Item '{serial_number}' is not in inventory")
        pickup_node = _map_shelf_id(location["shelf_id"])
        task, result = _dispatch_warehouse_task(serial_number, 1, pickup_node, gate)
        remove_inventory_item(serial_number)
        create_log(serial_number, "OUT", pickup_node, gate)
    except WarehouseDatabaseError as error:
        return jsonify({"error": str(error)}), 400

    return jsonify({"message": "Outbound task assigned successfully", "task": task, "result": result})


@app.route("/tasks/relocation", methods=["POST"])
def create_relocation_task():
    """Dispatch an item between shelves and update inventory atomically."""
    data = request.get_json(silent=True) or {}
    pickup_shelf = data.get("pickup_shelf", "")
    destination_shelf = data.get("destination_shelf", "")
    if not all((pickup_shelf, destination_shelf)):
        return jsonify({"error": "pickup_shelf and destination_shelf are required"}), 400

    try:
        inventory = get_item_at_shelf(pickup_shelf)
        if not inventory:
            raise WarehouseDatabaseError(f"Shelf '{pickup_shelf}' is empty")
        serial_number = inventory["item_id"]
        pickup_node = _map_shelf_id(pickup_shelf)
        destination_node = _map_shelf_id(destination_shelf)
        task, result = _dispatch_warehouse_task(serial_number, 2, pickup_node, destination_node)
        move_inventory_item(serial_number, pickup_shelf, destination_shelf)
        create_log(serial_number, "RELOCATION", pickup_node, destination_node)
    except WarehouseDatabaseError as error:
        return jsonify({"error": str(error)}), 400

    return jsonify({"message": "Relocation task assigned successfully", "task": task, "result": result})


@app.route("/warehouse/shelves")
def warehouse_shelves():
    """Shelf data for task-form dropdowns; availability is inventory-derived."""
    return jsonify(get_shelves())


@app.route("/warehouse/inventory")
def warehouse_inventory():
    """Current items for outbound-task selection."""
    return jsonify(get_inventory_items())


@app.route("/warehouse/monitor")
def warehouse_monitor():
    """Live warehouse stock, shelf, and log snapshot for the dashboard."""
    return jsonify(get_warehouse_monitor())

@app.route("/tasks")
def get_tasks():
    return jsonify(tasks)


@app.route("/robots")
def get_robots():
    return jsonify({robot_id: robot_state(robot_id) for robot_id in robots})


@app.route("/robots/<robot_id>/node", methods=["POST"])
def report_robot_node(robot_id):
    """HTTP test equivalent of an MQTT node report."""
    data = request.get_json(silent=True) or {}
    node_id = data.get("node_id", data.get("node"))
    if not isinstance(node_id, str):
        return jsonify({"error": "node_id is required"}), 400
    try:
        state = update_robot_node(robot_id, node_id.strip(), source="http")
        publish_robot_state(state)
        return jsonify(state)
    except ValueError as error:
        return jsonify({"error": str(error)}), 400


@app.route("/robots/<robot_id>/cancel-preview", methods=["POST"])
def preview_cancel_replan(robot_id):
    """Preview a virtual-node route to the next queued task without canceling it."""
    if robot_id not in robots:
        return jsonify({"error": f"Unknown robot ID: {robot_id}"}), 404

    robot = robots[robot_id]
    current_task = robot.get("current_task")
    if not current_task:
        return jsonify({"error": "Robot has no active task"}), 409
    if current_task.get("status") != "TO_PICKUP" or robot.get("has_payload"):
        return jsonify({"error": "Replanning preview is available only during pickup"}), 409

    pending_tasks = [task for task in robot["queue"] if task["id"] != current_task["id"]]
    if not pending_tasks:
        return jsonify({
            "preview_only": True,
            "next_task": None,
            "plan": None,
            "message": "There is no queued task to replan toward.",
        })

    edge = COMMAND_PLANNER.current_edge_for_robot(robot_id)
    at_known_parking_start = False
    if not edge and robot.get("last_node_update") is None:
        # The server may have restarted while the robot was manually returned
        # to its agreed parking pose; infer only that known initial edge.
        edge = COMMAND_PLANNER.initial_edge_from_parking(
            robot_id, robot["node"], current_task["PL"]
        )
        at_known_parking_start = edge is not None
    if not edge:
        return jsonify({"error": "No current RFID edge is available yet"}), 409
    distance_cm = 0 if at_known_parking_start else robot.get("distance_since_last_node_cm")
    if distance_cm is None:
        return jsonify({"error": "No encoder distance is available yet"}), 409

    try:
        plan = VIRTUAL_REPLANNER.plan_from_encoder(
            robot_id=robot_id,
            last_node=edge["last_node"],
            next_node=edge["next_node"],
            distance_from_last_cm=distance_cm,
            destination=pending_tasks[0]["PL"],
        )
    except ValueError as error:
        return jsonify({"error": str(error)}), 409

    return jsonify({
        "preview_only": True,
        "distance_is_estimate": not is_encoder_calibrated(robot_id),
        "next_task": pending_tasks[0],
        "plan": plan,
        "message": "Route preview only; no task was canceled and no movement command was sent.",
    })


@app.route("/robots/<robot_id>/cancel", methods=["POST"])
def cancel_and_replan(robot_id):
    """Cancel a pre-pickup task, promote its successor, and command its replanned route."""
    if robot_id not in robots:
        return jsonify({"error": f"Unknown robot ID: {robot_id}"}), 404

    robot = robots[robot_id]
    current_task = robot.get("current_task")
    if not current_task:
        return jsonify({"error": "Robot has no active task"}), 409
    if current_task.get("status") != "TO_PICKUP" or robot.get("has_payload"):
        return jsonify({"error": "A task can only be canceled during pickup"}), 409

    pending = [task for task in robot["queue"] if task["id"] != current_task["id"]]
    destination = pending[0]["PL"] if pending else "Parking_1"

    edge = COMMAND_PLANNER.current_edge_for_robot(robot_id)
    at_known_parking_start = False
    if not edge and robot.get("last_node_update") is None:
        edge = COMMAND_PLANNER.initial_edge_from_parking(robot_id, robot["node"], destination)
        at_known_parking_start = edge is not None
    if not edge:
        return jsonify({"error": "No current RFID edge is available yet"}), 409
    if not at_known_parking_start and not is_encoder_calibrated(robot_id):
        return jsonify({"error": "Encoder calibration is required before cancel-and-replan can move the AGV. Preview is still available."}), 409

    distance_cm = 0 if at_known_parking_start else robot.get("distance_since_last_node_cm")
    if distance_cm is None:
        return jsonify({"error": "No encoder distance is available yet"}), 409
    try:
        plan = VIRTUAL_REPLANNER.plan_from_encoder(
            robot_id, edge["last_node"], edge["next_node"], distance_cm, destination
        )
        if at_known_parking_start and robot["node"] == destination and distance_cm == 0:
            # The manually established startup pose is already the requested idle location.
            plan["first_action"] = "STOP"
            plan["first_reentry_node"] = destination
        if not plan.get("success") or not plan.get("first_action") or not plan.get("first_reentry_node"):
            return jsonify({"error": "The replanner could not produce a safe first movement"}), 409

        stop_command = {
            "action": "STOP", "task": "NONE", "current_node": robot["node"],
            "next_node": None, "goal": None, "path": [], "previous_node": edge.get("previous_node"),
        }
        publish_robot_command(robot_id, stop_command)
    except (ValueError, RuntimeError) as error:
        return jsonify({"error": str(error)}), 409

    try:
        result = cancel_current_task(
            robot_id,
            before_cancel=lambda task: rollback_scheduled_task(task),
        )
    except (ValueError, WarehouseDatabaseError) as error:
        return jsonify({"error": str(error)}), 409

    cancelled_id = result["cancelled_task"]["id"]
    tasks[:] = [task for task in tasks if task.get("id") != cancelled_id]

    next_task = result["next_task"]
    if next_task is None:
        if robot["node"] == destination and at_known_parking_start and distance_cm == 0:
            robot["idle_goal"] = None
            robot["status"] = "IDLE"
        else:
            robot["idle_goal"] = destination
            robot["status"] = "RETURNING_TO_PARKING"
        result["robot_state"] = robot_state(robot_id)
        command = {
            "action": plan["first_action"],
            "task": "IDLE",
            "current_node": edge["last_node"],
            "next_node": plan["first_reentry_node"],
            "goal": destination,
            "path": plan["path"],
            "previous_node": edge.get("previous_node"),
        }
    else:
        command = {
            "action": plan["first_action"],
            "task": "PICKUP",
            "current_node": edge["last_node"],
            "next_node": plan["first_reentry_node"],
            "goal": next_task["PL"],
            "path": plan["path"],
            "previous_node": edge.get("previous_node"),
        }
    robots[robot_id]["map_edge"] = (
        {
            "from_node": command["current_node"],
            "to_node": command["next_node"],
            "start_position": [coordinate / CM_PER_MAP_UNIT for coordinate in plan["state"]["current_position_cm"]],
            "distance_origin_cm": distance_cm,
        }
        if command.get("next_node") in nodes
        else None
    )
    robots[robot_id]["display_route"] = [
        route_node for route_node in command.get("path", [])
        if route_node in nodes
    ]
    result["robot_state"] = robot_state(robot_id)
    publish_robot_state(result["robot_state"])
    try:
        topic = publish_robot_command(robot_id, command)
    except RuntimeError as error:
        return jsonify({
            "error": f"Task was canceled and the next task was promoted, but its command could not be sent: {error}",
            "cancelled_task": result["cancelled_task"],
            "next_task": next_task,
            "robot_state": result["robot_state"],
            "plan": plan,
        }), 503

    return jsonify({
        "message": (
            "Task canceled; the robot is returning to Parking_1."
            if next_task is None
            else "Task canceled; the next queued task was replanned and dispatched."
        ),
        "cancelled_task": result["cancelled_task"],
        "next_task": next_task,
        "robot_state": result["robot_state"],
        "plan": plan,
        "command": command,
        "command_topic": topic,
    })


@app.route("/map")
def get_map():
    return jsonify({"nodes": nodes, "edges": edges})


@app.route("/camera-url")
def get_camera_url_route():
    """Return the most recent QR camera URL received over MQTT."""
    return jsonify({"url": get_camera_url()})


@app.route("/robot-events")
def robot_events():
    """Push state changes to the UI immediately after MQTT/HTTP updates."""
    subscriber = subscribe()

    def stream():
        try:
            yield "retry: 1000\n\n"
            while True:
                try:
                    event = subscriber.get(timeout=20)
                    yield f"event: {event['event']}\ndata: {json.dumps(event['data'])}\n\n"
                except Empty:
                    yield ": keepalive\n\n"
        finally:
            unsubscribe(subscriber)

    return Response(
        stream_with_context(stream()),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )



if __name__=="__main__":
    start_realtime_mqtt_gateway()
    serve(
        app,
        host="0.0.0.0",
        port=8000,
        # Each MJPEG camera feed is a long-lived response and occupies a
        # Waitress worker. Leave enough workers for both feeds, robot events,
        # and normal warehouse API requests.
        threads=int(os.environ.get("WAITRESS_THREADS", "16")),
    )
