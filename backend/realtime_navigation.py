"""Node-by-node command generation for RFID-guided AGVs.

The ESP32 stops at each RFID tag.  This module replans from that confirmed
node and returns exactly one command for the next edge of the route.
"""

from RealtimeReplanningService import RealtimeReplanningService
from new_warehouse_map import G, nodes
from robot_config import ROBOT_HOME_NODES


def movement_action_for_vectors(heading_vector, movement_vector, at_intersection=False):
    """Choose motion relative to the AGV's front-facing heading.

    A reverse move changes travel direction without changing the heading.
    Positive map Y is treated consistently with the warehouse map coordinates.
    """
    def unit_component(value):
        if abs(value) < 1e-6:
            return 0
        return 1 if value > 0 else -1

    heading = tuple(unit_component(value) for value in heading_vector)
    movement = tuple(unit_component(value) for value in movement_vector)
    if not any(movement):
        return "STOP"
    if heading == movement:
        return "FORWARD"
    if heading == (-movement[0], -movement[1]):
        return "TURN_BACK" if at_intersection else "BACKWARD"
    cross = heading[0] * movement[1] - heading[1] * movement[0]
    if cross > 0:
        return "RIGHT"
    if cross < 0:
        return "LEFT"
    return "STOP"


def forklift_task_for_route(task_stage, current_node, path):
    """Trigger pickup/dropoff when an RFID report places the robot before its goal."""
    if (
        task_stage in {"PICKUP", "DROPOFF"}
        and len(path) >= 2
        and current_node == path[-2]
    ):
        return task_stage
    return "NONE"


class WarehouseGraphAdapter:
    """Expose the warehouse NetworkX graph through the replanner's API."""

    def __init__(self, graph):
        self._graph = graph

    @property
    def nodes(self):
        return self._graph.nodes

    def has_node(self, node):
        return self._graph.has_node(node)

    def get_position(self, node):
        return self._graph.nodes[node]["pos"]

    def get_neighbors(self, node):
        return [
            (neighbor, data.get("weight", 1))
            for neighbor, data in self._graph[node].items()
        ]


class NodeCommandPlanner:
    """Replan an AGV route every time it confirms an RFID node."""

    ESP32_ACTIONS = {
        "LEFT": "LEFT",
        "RIGHT": "RIGHT",
        "FORWARD": "FORWARD",
        "BACK": "BACKWARD",
        "BACKWARD": "BACKWARD",
        # Accept old internal planner vocabulary, but send an explicit reverse
        # movement action over MQTT. Reverse travel does not rotate the AGV.
        "TURN_BACK": "TURN_BACK",
        "STOP": "STOP",
    }

    def __init__(self):
        graph = WarehouseGraphAdapter(G)
        self._replanner = RealtimeReplanningService(graph)
        self._last_confirmed_node = {}
        self._previous_confirmed_node = {}
        self._last_command = {}

    def command_for_node(self, robot_id, current_node, goal, task_stage="NONE", heading_vector=None):
        """Return one ESP32 command after *robot_id* reaches *current_node*.

        A duplicate QoS-1 MQTT node message resends the previous command rather
        than treating the robot as having reached the next node twice.
        """
        if current_node not in nodes or goal not in nodes:
            raise ValueError("Current node and goal must be warehouse map nodes")

        last_confirmed_node = self._last_confirmed_node.get(robot_id)
        cached_command = self._last_command.get(robot_id)
        if (
            last_confirmed_node == current_node
            and cached_command
            and cached_command["goal"] == goal
            and cached_command["task"] == task_stage
        ):
            return dict(cached_command)

        previous_node = (
            last_confirmed_node
            if last_confirmed_node != current_node
            else self._previous_confirmed_node.get(robot_id)
        )
        plan = self._replanner.plan(current_node, goal)
        if plan["status"] != "success" or not plan["path"]:
            command = self._command("STOP", current_node, goal, [], previous_node, task_stage)
        else:
            path = plan["path"]
            if len(path) == 1:
                command = self._command("STOP", current_node, goal, path, previous_node, task_stage)
            else:
                # A* chooses the route without traffic or heading costs. Only
                # command generation compares its first edge with actual facing.
                if heading_vector is None:
                    if previous_node and G.has_edge(previous_node, current_node):
                        heading_vector = self._unit_vector(previous_node, current_node)
                    else:
                        heading_vector = self._unit_vector(current_node, path[1])
                movement_vector = self._unit_vector(current_node, path[1])
                action = movement_action_for_vectors(
                    heading_vector,
                    movement_vector,
                    at_intersection=G.degree[current_node] >= 3,
                )
                command = self._command(action, current_node, goal, path, previous_node, task_stage)

        if last_confirmed_node != current_node:
            self._previous_confirmed_node[robot_id] = last_confirmed_node
            self._last_confirmed_node[robot_id] = current_node
        self._last_command[robot_id] = command
        return dict(command)

    @staticmethod
    def _unit_vector(from_node, to_node):
        x1, y1 = nodes[from_node]
        x2, y2 = nodes[to_node]
        return (
            1 if x2 > x1 else -1 if x2 < x1 else 0,
            1 if y2 > y1 else -1 if y2 < y1 else 0,
        )

    def current_edge_for_robot(self, robot_id):
        """Return the last RFID edge command used to interpret encoder progress."""
        command = self._last_command.get(robot_id)
        if not command or not command.get("next_node"):
            return None
        return {
            "last_node": command["current_node"],
            "next_node": command["next_node"],
            "previous_node": command.get("previous_node"),
        }

    def initial_edge_from_parking(self, robot_id, current_node, goal):
        """Infer the first edge only for the confirmed parking/start orientation."""
        parking_node = ROBOT_HOME_NODES.get(robot_id)
        parking_junction = f"{parking_node}_J" if parking_node else None
        if current_node != parking_node:
            return None
        next_node = parking_junction
        if not G.has_edge(current_node, next_node) or goal not in nodes:
            return None
        return {
            "last_node": current_node,
            "next_node": next_node,
            "previous_node": None,
        }

    def _command(self, action, current_node, goal, path, previous_node, task_stage):
        action = self.ESP32_ACTIONS.get(action.upper(), "STOP")
        return {
            "action": action,
            "task": task_stage,
            "forklift_task": forklift_task_for_route(task_stage, current_node, path),
            "current_node": current_node,
            "next_node": path[1] if len(path) > 1 else None,
            "goal": goal,
            "path": path,
            "previous_node": previous_node,
        }
