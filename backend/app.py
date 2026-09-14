import json
from queue import Empty
from flask import Flask, Response, request, jsonify, stream_with_context
from flask_cors import CORS
from waitress import serve
from scheduler.scheduler import dispatch_task, robot_state, robots, update_robot_node
from new_warehouse_map import edges, nodes
from realtime_mqtt_gateway import start_realtime_mqtt_gateway
from robot_events import publish_robot_state, subscribe, unsubscribe
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
)
from database.seed import seed_shelves

app = Flask(__name__)

CORS(app)

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
        publish_robot_state(result["robot_state"])
    except WarehouseDatabaseError as error:
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


@app.route("/map")
def get_map():
    return jsonify({"nodes": nodes, "edges": edges})


@app.route("/robot-events")
def robot_events():
    """Push state changes to the UI immediately after MQTT/HTTP updates."""
    subscriber = subscribe()

    def stream():
        try:
            yield "retry: 1000\n\n"
            while True:
                try:
                    state = subscriber.get(timeout=20)
                    yield f"event: robot-state\ndata: {json.dumps(state)}\n\n"
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
        port=8000
    )
