"""Node-by-node command generation for RFID-guided AGVs.

The ESP32 stops at each RFID tag.  This module replans from that confirmed
node and returns exactly one command for the next edge of the route.
"""

from RealtimeReplanningService import RealtimeReplanningService
from new_warehouse_map import G, nodes


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
        self._last_confirmed_node = {}
        self._last_command = {}

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
