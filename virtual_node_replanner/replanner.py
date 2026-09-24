"""The existing Virtual Node replanning behavior, isolated from Django/services."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone

from .astar import astar
from .directions import DirectionGenerator


DEFAULT_BACKWARD_PENALTY_CM = 10.0


def _backward_penalty(graph, state: AGVState, backward_penalty_cm: float) -> float:
    """Return the existing planning-cost penalty when previous is behind heading."""
    previous_position = graph.get_position(state.previous_node)
    vector_to_previous = (
        previous_position[0] - state.current_position[0],
        previous_position[1] - state.current_position[1],
    )
    heading_radians = math.radians(state.heading)
    return backward_penalty_cm if (
        math.cos(heading_radians) * vector_to_previous[0]
        + math.sin(heading_radians) * vector_to_previous[1]
    ) < 0 else 0.0


@dataclass(frozen=True)
class AGVState:
    """State values the existing replanner receives from its caller."""

    robot_id: str
    previous_node: str
    next_node: str
    destination: str
    heading: float
    distance_from_previous_cm: float
    distance_to_next_cm: float
    current_position: tuple[float, float]
    simulation_time: float
    conflict_detected: bool = False


class VirtualNodeReplanner:
    """Clone a graph, insert a temporary robot node, then run the existing A*."""

    name = "virtual_node"

    def __init__(self, graph, blocked_edges=None, blocked_nodes=None):
        self.graph = graph
        self.blocked_edges = blocked_edges or set()
        self.blocked_nodes = blocked_nodes or set()

    def replan(self, state: AGVState, backward_penalty_cm: float) -> dict:
        total_started = time.perf_counter()
        preparation_started = total_started
        working_graph = self.graph.clone()
        for node_a, node_b in self.blocked_edges:
            working_graph.remove_edge(node_a, node_b)
        for blocked_node in self.blocked_nodes:
            for neighbor, _ in list(working_graph.graph.get(blocked_node, [])):
                working_graph.remove_edge(blocked_node, neighbor)
        temporary_node = "TEMP_ROBOT_%s" % state.robot_id
        working_graph.add_temp_node(temporary_node, *state.current_position)
        working_graph.connect_temp_node(
            state.previous_node, state.next_node, temporary_node
        )
        applied_penalty = _backward_penalty(self.graph, state, backward_penalty_cm)
        if applied_penalty:
            # Temporary-to-previous is the backward candidate. Preserve the
            # existing path-planning cost behavior.
            working_graph.graph[temporary_node] = [
                (node, cost + applied_penalty if node == state.previous_node else cost)
                for node, cost in working_graph.graph[temporary_node]
            ]
            working_graph.graph[state.previous_node] = [
                (node, cost + applied_penalty if node == temporary_node else cost)
                for node, cost in working_graph.graph[state.previous_node]
            ]
        preparation_ms = (time.perf_counter() - preparation_started) * 1000

        astar_started_at = datetime.now(timezone.utc).isoformat()
        astar_started = time.perf_counter()
        result = astar(working_graph, temporary_node, state.destination)
        astar_ms = (time.perf_counter() - astar_started) * 1000
        astar_finished_at = datetime.now(timezone.utc).isoformat()
        processing_started = time.perf_counter()
        directions = DirectionGenerator(working_graph).generate(result["path"]) if result else []
        processing_ms = (time.perf_counter() - processing_started) * 1000
        cleanup_started = time.perf_counter()
        working_graph.remove_temp_node(temporary_node)
        cleanup_ms = (time.perf_counter() - cleanup_started) * 1000
        total_ms = (time.perf_counter() - total_started) * 1000
        component_total_ms = preparation_ms + astar_ms + processing_ms + cleanup_ms
        overhead_ms = max(0.0, total_ms - component_total_ms)

        return {
            "success": result is not None,
            "path": result["path"] if result else [],
            "path_length_cm": result["distance"] if result else None,
            "path_node_count": len([node for node in result["path"] if node != temporary_node]) if result else 0,
            "directions": directions,
            "selected_candidate": "virtual_node",
            "preparation_time_ms": preparation_ms,
            "astar_time_ms": astar_ms,
            "candidate_evaluation_time_ms": 0.0,
            "path_processing_time_ms": processing_ms,
            "cleanup_time_ms": cleanup_ms,
            "unattributed_overhead_time_ms": overhead_ms,
            "total_time_ms": total_ms,
            "distance_to_previous_cm": state.distance_from_previous_cm,
            "distance_to_next_cm": state.distance_to_next_cm,
            "backward_penalty_cm": applied_penalty,
            "configured_backward_penalty_cm": backward_penalty_cm,
            "backward_movement": bool(applied_penalty),
            "previous_candidate_cost": state.distance_from_previous_cm + applied_penalty,
            "next_candidate_cost": state.distance_to_next_cm,
            "nodes_explored": result["nodes_explored"] if result else 0,
            "astar_iterations": result["astar_iterations"] if result else 0,
            "astar_executions": [{
                "start_node": temporary_node, "goal_node": state.destination,
                "start_node_type": "temporary_virtual", "calculation_started_at": astar_started_at,
                "calculation_finished_at": astar_finished_at, "calculation_time_ms": astar_ms,
                "nodes_explored_count": result["nodes_explored"] if result else 0,
                "astar_iterations": result["astar_iterations"] if result else 0,
                "exploration_order": result["exploration_order"] if result else [],
                "closed_nodes_count": result["closed_nodes_count"] if result else 0,
                "neighbor_evaluations": result["neighbor_evaluations"] if result else 0,
                "maximum_open_set_size": result["maximum_open_set_size"] if result else 0,
                "final_path": result["path"] if result else [],
                "path_distance_cm": result["distance"] if result else None,
                "success": result is not None,
            }],
            "temporary_graph": {
                "temporary_node_id": temporary_node, "coordinates": state.current_position,
                "previous_node": state.previous_node, "next_node": state.next_node,
                "previous_to_temporary_cm": state.distance_from_previous_cm,
                "temporary_to_next_cm": state.distance_to_next_cm,
                "temporary_node_count": 1, "temporary_edge_count": 2,
                "graph_copy_creation_time_ms": preparation_ms,
            },
        }
