"""Bridge the warehouse NetworkX map and live encoder state to the virtual replanner."""
import copy
import math
import time

from new_warehouse_map import G, nodes
from virtual_node_replanner import AGVState, VirtualNodeReplanner


CM_PER_MAP_UNIT = 2.54
BACKWARD_PENALTY_CM = 10.0


class WarehouseGraphAdapter:
    """Expose the live warehouse map using the replanner's graph interface."""

    def __init__(self):
        self.nodes = {
            node: {"x": position[0] * CM_PER_MAP_UNIT, "y": position[1] * CM_PER_MAP_UNIT}
            for node, position in nodes.items()
        }
        self.graph = {node: [] for node in self.nodes}
        for node_a, node_b, edge_data in G.edges(data=True):
            cost_cm = float(edge_data.get("weight", 1)) * CM_PER_MAP_UNIT
            self.graph[node_a].append((node_b, cost_cm))
            self.graph[node_b].append((node_a, cost_cm))

    def clone(self):
        return copy.deepcopy(self)

    def has_node(self, node):
        return node in self.nodes

    def get_position(self, node):
        position = self.nodes[node]
        return position["x"], position["y"]

    def get_neighbors(self, node):
        return self.graph[node]

    def remove_edge(self, node_a, node_b):
        if node_a in self.graph:
            self.graph[node_a] = [entry for entry in self.graph[node_a] if entry[0] != node_b]
        if node_b in self.graph:
            self.graph[node_b] = [entry for entry in self.graph[node_b] if entry[0] != node_a]

    def add_temp_node(self, node, x, y):
        self.nodes[node] = {"x": x, "y": y}
        self.graph[node] = []

    def connect_temp_node(self, node_a, node_b, temporary_node):
        temp_x, temp_y = self.get_position(temporary_node)
        for endpoint in (node_a, node_b):
            end_x, end_y = self.get_position(endpoint)
            cost = abs(temp_x - end_x) + abs(temp_y - end_y)
            self.graph[temporary_node].append((endpoint, cost))
            self.graph[endpoint].append((temporary_node, cost))

    def remove_temp_node(self, node):
        if node not in self.graph:
            return
        for neighbor, _cost in list(self.graph[node]):
            self.remove_edge(node, neighbor)
        self.graph.pop(node, None)
        self.nodes.pop(node, None)


class LiveVirtualReplanner:
    """Create a virtual-node route preview from the latest RFID edge and encoder."""

    def __init__(self):
        self._replanner = VirtualNodeReplanner(WarehouseGraphAdapter())

    def plan_from_encoder(self, robot_id, last_node, next_node, distance_from_last_cm, destination):
        if last_node not in nodes or next_node not in nodes or destination not in nodes:
            raise ValueError("Current edge and destination must be warehouse map nodes")
        if not G.has_edge(last_node, next_node):
            raise ValueError(f"No warehouse edge from {last_node} to {next_node}")
        if isinstance(distance_from_last_cm, bool) or not isinstance(distance_from_last_cm, (int, float)):
            raise ValueError("Encoder distance is not available")
        if not math.isfinite(distance_from_last_cm) or distance_from_last_cm < 0:
            raise ValueError("Encoder distance must be a finite non-negative number")

        x1, y1 = nodes[last_node]
        x2, y2 = nodes[next_node]
        dx = (x2 - x1) * CM_PER_MAP_UNIT
        dy = (y2 - y1) * CM_PER_MAP_UNIT
        edge_length_cm = abs(dx) + abs(dy)
        if edge_length_cm <= 0:
            raise ValueError("Current RFID edge has no physical length")
        if distance_from_last_cm > edge_length_cm + 5:
            raise ValueError("Encoder distance exceeds the current RFID edge; wait for a fresh node report")

        progress_cm = min(float(distance_from_last_cm), edge_length_cm)
        current_position = (
            x1 * CM_PER_MAP_UNIT + (dx / edge_length_cm) * progress_cm,
            y1 * CM_PER_MAP_UNIT + (dy / edge_length_cm) * progress_cm,
        )
        heading = math.degrees(math.atan2(dy, dx))
        state = AGVState(
            robot_id=robot_id,
            previous_node=last_node,
            next_node=next_node,
            destination=destination,
            heading=heading,
            distance_from_previous_cm=progress_cm,
            distance_to_next_cm=max(0.0, edge_length_cm - progress_cm),
            current_position=current_position,
            simulation_time=time.monotonic(),
        )
        result = self._replanner.replan(state, BACKWARD_PENALTY_CM)
        if result["success"]:
            result["first_reentry_node"] = result["path"][1] if len(result["path"]) > 1 else None
            result["first_action"] = (
                "FORWARD" if result["first_reentry_node"] == next_node
                else "TURN_BACK" if result["first_reentry_node"] == last_node
                else None
            )
        result["state"] = {
            "robot_id": robot_id,
            "last_rfid_node": last_node,
            "next_rfid_node": next_node,
            "destination": destination,
            "heading_degrees": heading,
            "distance_from_last_cm": progress_cm,
            "distance_to_next_cm": max(0.0, edge_length_cm - progress_cm),
            "current_position_cm": current_position,
        }
        return result
