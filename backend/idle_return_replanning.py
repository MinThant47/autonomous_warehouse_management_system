"""Interrupt an idle parking return when new work is assigned to that AGV."""

from encoder_distance import is_encoder_calibrated
from new_warehouse_map import nodes
from scheduler.scheduler import robot_state, robots


def replan_interrupted_idle_return(result, command_planner, replanner, publish_command):
    """Replan the newly assigned task from the robot's latest encoder position.

    A task remains assigned if replanning or MQTT publishing fails. In that
    case the old parking motion is stopped where possible and the result
    includes an error for the caller to report.
    """
    if not result.get("interrupted_idle_return"):
        return None

    robot_id = result["assigned_robot"]
    robot = robots[robot_id]
    task = robot.get("current_task")
    if not task:
        return {"error": "The assigned robot has no active task to replan."}

    edge = command_planner.current_edge_for_robot(robot_id)
    # Cancel-and-replan commands are published directly, so the map edge is
    # the authoritative latest command even when the node planner cache still
    # contains the route from before the idle return began.
    map_edge = robot.get("map_edge")
    if map_edge and map_edge.get("from_node") in nodes and map_edge.get("to_node") in nodes:
        previous_edge = edge or {}
        edge = {
            "last_node": map_edge["from_node"],
            "next_node": map_edge["to_node"],
            "previous_node": (
                previous_edge.get("previous_node")
                if previous_edge.get("last_node") == map_edge["from_node"]
                and previous_edge.get("next_node") == map_edge["to_node"]
                else None
            ),
        }
    at_known_parking_start = False
    if not edge and robot.get("last_node_update") is None:
        edge = command_planner.initial_edge_from_parking(robot_id, robot["node"], task["PL"])
        at_known_parking_start = edge is not None

    distance_cm = 0 if at_known_parking_start else robot.get("distance_since_last_node_cm")
    error = None
    plan = None
    if not edge:
        error = "No current RFID edge is available to replan from."
    elif distance_cm is None:
        error = "No encoder distance is available to replan from."
    elif not at_known_parking_start and not is_encoder_calibrated(robot_id):
        error = "Encoder calibration is required to interrupt the parking return safely."
    else:
        try:
            plan = replanner.plan_from_encoder(
                robot_id,
                edge["last_node"],
                edge["next_node"],
                distance_cm,
                task["PL"],
            )
            if not plan.get("success") or not plan.get("first_action") or not plan.get("first_reentry_node"):
                error = "The replanner could not produce a safe first movement."
        except (ValueError, RuntimeError) as exception:
            error = str(exception)

    stop_command = {
        "action": "STOP",
        "task": "NONE",
        "current_node": robot["node"],
        "next_node": None,
        "goal": None,
        "path": [],
        "previous_node": edge.get("previous_node") if edge else None,
    }
    try:
        publish_command(robot_id, stop_command)
    except RuntimeError as exception:
        error = error or f"Could not stop the previous parking command: {exception}"

    if error:
        robot["map_edge"] = None
        robot["display_route"] = []
        result["robot_state"] = robot_state(robot_id)
        return {"error": error, "robot_state": result["robot_state"], "plan": plan}

    command = {
        "action": plan["first_action"],
        "task": "PICKUP",
        "current_node": edge["last_node"],
        "next_node": plan["first_reentry_node"],
        "goal": task["PL"],
        "path": plan["path"],
        "previous_node": edge.get("previous_node"),
    }
    robot["map_edge"] = (
        {
            "from_node": command["current_node"],
            "to_node": command["next_node"],
            "start_position": [
                coordinate / 2.54
                for coordinate in plan["state"]["current_position_cm"]
            ],
            "distance_origin_cm": distance_cm,
        }
        if command["next_node"] in nodes
        else None
    )
    robot["display_route"] = [node for node in command["path"] if node in nodes]

    try:
        topic = publish_command(robot_id, command)
    except RuntimeError as exception:
        result["robot_state"] = robot_state(robot_id)
        return {
            "error": f"The parking route was stopped, but the new pickup command could not be sent: {exception}",
            "robot_state": result["robot_state"],
            "plan": plan,
            "command": command,
        }

    result["robot_state"] = robot_state(robot_id)
    return {
        "plan": plan,
        "command": command,
        "command_topic": topic,
        "robot_state": result["robot_state"],
    }
