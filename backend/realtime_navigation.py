"""Node-by-node command generation for RFID-guided AGVs.

The ESP32 stops at each RFID tag.  This module replans from that confirmed
node and returns exactly one command for the next edge of the route.
"""

import math
from pathlib import Path
import sys

from RealtimeReplanningService import RealtimeReplanningService
from new_warehouse_map import G, nodes

# The extracted virtual-node package lives beside backend/ so both the
# standalone module and the live backend use the same implementation.
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from virtual_node_replanner import (
    AGVState,
    DEFAULT_BACKWARD_PENALTY_CM,
    Graph,
    VirtualNodeReplanner,
)


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
        "BACK": "TURN_BACK",
        "BACKWARD": "TURN_BACK",
        "STOP": "STOP",
    }

    def __init__(self):
        graph = WarehouseGraphAdapter(G)
        self._replanner = RealtimeReplanningService(graph)
        virtual_graph = Graph(str(_PROJECT_ROOT / "virtual_node_replanner" / "warehouse_map.json"))
        self._virtual_replanner = VirtualNodeReplanner(virtual_graph)
        self._last_confirmed_node = {}
        self._last_command = {}

    def command_for_virtual_state(self, state):
        """Plan from an on-edge robot pose supplied by localization hardware.

        ``state`` must contain previous_node, next_node, destination, heading,
        current_position, and optionally distance fields and simulation_time.
        The current MQTT node-only protocol continues to use command_for_node.
        """
        agv_state = AGVState(
            robot_id=str(state["robot_id"]),
            previous_node=state["previous_node"],
            next_node=state["next_node"],
            destination=state["destination"],
            heading=float(state["heading"]),
            distance_from_previous_cm=float(state.get("distance_from_previous_cm", 0.0)),
            distance_to_next_cm=float(state.get("distance_to_next_cm", 0.0)),
            current_position=tuple(map(float, state["current_position"])),
            simulation_time=float(state.get("simulation_time", 0.0)),
        )
        if len(agv_state.current_position) != 2:
            raise ValueError("current_position must contain exactly two coordinates")
        if not all(isinstance(node, str) for node in (agv_state.previous_node, agv_state.next_node, agv_state.destination)):
            raise ValueError("Virtual replanning nodes must be strings")
        if agv_state.previous_node not in nodes or agv_state.next_node not in nodes or agv_state.destination not in nodes:
            raise ValueError("Virtual replanning edge and destination must be warehouse map nodes")
        if not G.has_edge(agv_state.previous_node, agv_state.next_node):
            raise ValueError("Virtual replanning endpoints must describe a connected warehouse edge")

        result = self._virtual_replanner.replan(agv_state, DEFAULT_BACKWARD_PENALTY_CM)
        path = [node for node in result["path"] if not node.startswith("TEMP_ROBOT_")]
        if not result["success"] or not path:
            action = "STOP"
        else:
            dx = nodes[path[0]][0] - agv_state.current_position[0]
            dy = nodes[path[0]][1] - agv_state.current_position[1]
            desired_heading = math.degrees(math.atan2(dy, dx))
            turn = (desired_heading - agv_state.heading + 180.0) % 360.0 - 180.0
            if abs(turn) < 45.0:
                action = "FORWARD"
            elif abs(turn) > 135.0:
                action = "TURN_BACK"
            else:
                # Warehouse map y increases downward, matching the existing
                # direction generator's cross-product turn convention.
                action = "RIGHT" if turn > 0 else "LEFT"
        return {
            "action": action,
            "current_node": state.get("current_node"),
            "next_node": path[0] if path else None,
            "goal": agv_state.destination,
            "path": path,
            "previous_node": agv_state.previous_node,
            "planning": result,
        }

    def command_for_node(self, robot_id, current_node, goal):
        """Return one ESP32 command after *robot_id* reaches *current_node*.

        A duplicate QoS-1 MQTT node message resends the previous command rather
        than treating the robot as having reached the next node twice.
        """
        if current_node not in nodes or goal not in nodes:
            raise ValueError("Current node and goal must be warehouse map nodes")

        if self._last_confirmed_node.get(robot_id) == current_node:
            return dict(self._last_command[robot_id])

        previous_node = self._last_confirmed_node.get(robot_id)
        plan = self._replanner.plan(current_node, goal)
        if plan["status"] != "success" or not plan["path"]:
            command = self._command("STOP", current_node, goal, [], previous_node)
        else:
            path = plan["path"]
            if len(path) == 1:
                command = self._command("STOP", current_node, goal, path, previous_node)
            elif previous_node and G.has_edge(previous_node, current_node):
                action = self._replanner.direction_generator.get_action(
                    previous_node, current_node, path[1]
                )
                command = self._command(action, current_node, goal, path, previous_node)
            else:
                # The first tag after boot has no incoming-edge history. The
                # hardware is already aligned with its starting line, so let it
                # leave the node; all later junction commands use real heading.
                command = self._command("FORWARD", current_node, goal, path, previous_node)

        self._last_confirmed_node[robot_id] = current_node
        self._last_command[robot_id] = command
        return dict(command)

    def _command(self, action, current_node, goal, path, previous_node):
        action = self.ESP32_ACTIONS.get(action.upper(), "STOP")
        return {
            "action": action,
            "current_node": current_node,
            "next_node": path[1] if len(path) > 1 else None,
            "goal": goal,
            "path": path,
            "previous_node": previous_node,
        }
