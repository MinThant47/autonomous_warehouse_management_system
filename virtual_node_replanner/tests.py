"""Focused parity and graph-integrity tests for the extracted implementation."""
from pathlib import Path
import copy
import unittest

from planner.algorithms.graph import Graph as OriginalGraph
from planner.services.experiment_service import AGVState as OriginalState
from planner.services.experiment_service import VirtualNodeReplanner as OriginalReplanner

from . import AGVState, Graph, VirtualNodeReplanner


MAP_PATH = Path(__file__).with_name("warehouse_map.json")
SOURCE_MAP_PATH = Path(__file__).parents[1] / "data" / "warehouse_map.json"


def stable_result(result):
    """Exclude wall-clock timings while comparing behavioral outputs."""
    stable = {
        key: result[key]
        for key in (
            "success", "path", "path_length_cm", "path_node_count", "directions",
            "selected_candidate", "distance_to_previous_cm", "distance_to_next_cm",
            "backward_penalty_cm", "configured_backward_penalty_cm", "backward_movement",
            "previous_candidate_cost", "next_candidate_cost", "nodes_explored", "astar_iterations",
            "temporary_graph",
        )
    }
    stable["temporary_graph"] = {
        key: value for key, value in result["temporary_graph"].items()
        if key != "graph_copy_creation_time_ms"
    }
    return stable


class VirtualNodeExtractionTests(unittest.TestCase):
    def setUp(self):
        self.graph = Graph(str(MAP_PATH))
        self.original_graph = OriginalGraph(str(SOURCE_MAP_PATH))

    def state(self, robot_id="AGV_001", position=(54.0, 23.0), heading=90.0, destination="Green_7_J"):
        return AGVState(
            robot_id, "Parking_1_J", "Middle_1_J", destination, heading,
            5.0, 22.0, position, 0.0,
        )

    def test_matches_existing_replanner_for_midpoint_and_heading_cases(self):
        scenarios = (
            self.state(position=(54.0, 23.0), heading=90.0),
            self.state(position=(54.0, 44.9), heading=90.0),
            self.state(position=(54.0, 18.0), heading=90.0),
            self.state(position=(54.0, 23.0), heading=270.0),
            self.state(position=(54.0, 23.0), heading=90.0, destination="Gate_1"),
        )
        for index, state in enumerate(scenarios):
            with self.subTest(index=index):
                extracted_state = state
                original_state = OriginalState(**state.__dict__)
                actual = VirtualNodeReplanner(self.graph).replan(extracted_state, 10.0)
                expected = OriginalReplanner(self.original_graph).replan(original_state, 10.0)
                self.assertEqual(stable_result(actual), stable_result(expected))

    def test_original_graph_is_unchanged(self):
        before = copy.deepcopy((self.graph.nodes, self.graph.original_graph, self.graph.graph))
        state = self.state()
        VirtualNodeReplanner(self.graph).replan(state, 10.0)
        after = (self.graph.nodes, self.graph.original_graph, self.graph.graph)
        self.assertEqual(before, after)
        self.assertNotIn("TEMP_ROBOT_AGV_001", self.graph.nodes)

    def test_result_path_starts_at_temporary_node_and_keeps_that_node(self):
        result = VirtualNodeReplanner(self.graph).replan(self.state(), 10.0)
        self.assertTrue(result["success"])
        self.assertEqual(result["path"][0], "TEMP_ROBOT_AGV_001")
        self.assertEqual(result["astar_executions"][0]["start_node"], "TEMP_ROBOT_AGV_001")
        self.assertEqual(result["astar_executions"][0]["goal_node"], "Green_7_J")

    def test_unreachable_destination_returns_existing_no_path_shape(self):
        # Remove every edge incident to the destination in the input graph.
        for neighbor, _ in list(self.graph.graph["Gate_1"]):
            self.graph.remove_edge("Gate_1", neighbor)
        result = VirtualNodeReplanner(self.graph).replan(self.state(destination="Gate_1"), 10.0)
        self.assertFalse(result["success"])
        self.assertEqual(result["path"], [])
        self.assertIsNone(result["path_length_cm"])


if __name__ == "__main__":
    unittest.main()
