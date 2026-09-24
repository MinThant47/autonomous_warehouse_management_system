from planner.algorithms.astar import astar

from planner.algorithms.directions import DirectionGenerator

class RealtimeReplanningService:

    def __init__(self, graph):
        self.graph = graph
        self.direction_generator = DirectionGenerator(self.graph)
    # ========================================================
    # Main Replanning Function
    # ========================================================
    def plan(self, start, goal):

        print()
        print("========================================")
        print("[PLAN] INITIAL A* PATH PLANNING")
        print("========================================")

        print(f"[PLAN] Start : {start}")
        print(f"[PLAN] Goal  : {goal}")

        # ====================================================
        # A*
        # ====================================================

        result = astar(
            self.graph,
            start,
            goal
        )

        if result is None:

            print(
                f"[PLAN] No path from {start} to {goal}"
            )

            return {
                "status": "error",
                "message": "No path found",
                "path": [],
                "distance": 0,
                "directions": []
            }

        # ====================================================
        # Generate Directions
        # ====================================================

        directions = self.direction_generator.generate(
            result["path"]
        )

        # ====================================================
        # Debug
        # ====================================================

        print()
        print("----------------------------------------")

        print(f"[PLAN] Path:")
        print(result["path"])

        print(f"[PLAN] Distance:")
        print(result["distance"])

        print(f"[PLAN] Directions:")
        print(directions)

        print("----------------------------------------")

        # ====================================================
        # Return
        # ====================================================

        return {
            "status": "success",
            "path": result["path"],
            "distance": result["distance"],
            "directions": directions,
            "search_metrics": self._search_metrics(result),
        }

    def replan(
        self,
        last_node,
        next_node,
        distance_from_last,
        edge_distance,
        goal
    ):

        print()
        print("========================================")
        print("[REPLAN] REAL-TIME REPLANNING")
        print("========================================")

        print(f"[REPLAN] Last RFID Node : {last_node}")
        print(f"[REPLAN] Next RFID Node : {next_node}")
        print(
            f"[REPLAN] Distance from {last_node}: "
            f"{distance_from_last} cm"
        )
        print(
            f"[REPLAN] Edge distance: "
            f"{edge_distance} cm"
        )
        print(f"[REPLAN] Goal: {goal}")

        # ====================================================
        # Calculate distance to both RFID nodes
        # ====================================================

        distance_to_last = distance_from_last

        distance_to_next = (
            edge_distance - distance_from_last
        )

        # Safety
        if distance_to_next < 0:
            distance_to_next = 0

        print()
        print(
            f"[REPLAN] Distance to {last_node}: "
            f"{distance_to_last} cm"
        )

        print(
            f"[REPLAN] Distance to {next_node}: "
            f"{distance_to_next} cm"
        )

        # ====================================================
        # A* : Last Node -> Goal
        # ====================================================

        path_from_last = astar(
            self.graph,
            last_node,
            goal
        )

        if path_from_last is None:

            print(
                f"[REPLAN] No path from "
                f"{last_node} to {goal}"
            )

            cost_from_last = float("inf")

        else:

            cost_from_last = (
                distance_to_last
                + path_from_last["distance"]
            )

        # ====================================================
        # A* : Next Node -> Goal
        # ====================================================

        path_from_next = astar(
            self.graph,
            next_node,
            goal
        )

        if path_from_next is None:

            print(
                f"[REPLAN] No path from "
                f"{next_node} to {goal}"
            )

            cost_from_next = float("inf")

        else:

            cost_from_next = (
                distance_to_next
                + path_from_next["distance"]
            )

        # ====================================================
        # Print Comparison
        # ====================================================

        print()
        print("----------------------------------------")

        print(
            f"[REPLAN] Cost via {last_node}: "
            f"{cost_from_last}"
        )

        print(
            f"[REPLAN] Cost via {next_node}: "
            f"{cost_from_next}"
        )

        print("----------------------------------------")

        # ====================================================
        # Select Better Re-entry Node
        # ====================================================

        if cost_from_last <= cost_from_next:

            selected_node = last_node
            selected_path = path_from_last
            selected_cost = cost_from_last

            return_to_node = True
            action = "BACKWARD"

        else:

            selected_node = next_node
            selected_path = path_from_next
            selected_cost = cost_from_next

            return_to_node = False
            action = "FORWARD"

        # ====================================================
        # Final Result
        # ====================================================

        if selected_path is None:

            print(
                "[REPLAN] ERROR: No valid path."
            )

            return {
                "status": "error",
                "message": "No valid path found."
            }

        print()
        print(
            f"[REPLAN] Selected node: "
            f"{selected_node}"
        )

        print(
            f"[REPLAN] Action: "
            f"{action}"
        )

        print(
            f"[REPLAN] Path: "
            f"{selected_path['path']}"
        )

        print(
            f"[REPLAN] Total estimated cost: "
            f"{selected_cost}"
        )

        print(
            "========================================"
        )

        return {
            "status": "success",
            "replanning": True,

            "last_node": last_node,
            "next_node": next_node,

            "replan_node": selected_node,

            "action": action,

            "return_to_node": return_to_node,

            "distance_to_last": distance_to_last,
            "distance_to_next": distance_to_next,

            "astar_distance": selected_path["distance"],

            "estimated_total_cost": selected_cost,

            "search_metrics": self._search_metrics(selected_path),

            "path": selected_path["path"]
        }

    @staticmethod
    def _search_metrics(result):
        """Expose the A* diagnostics without changing route selection."""
        return {
            "nodes_explored": result.get("nodes_explored", 0),
            "astar_iterations": result.get("astar_iterations", 0),
            "exploration_order": result.get("exploration_order", []),
            "closed_nodes_count": result.get("closed_nodes_count", 0),
            "neighbor_evaluations": result.get("neighbor_evaluations", 0),
            "maximum_open_set_size": result.get("maximum_open_set_size", 0),
        }
