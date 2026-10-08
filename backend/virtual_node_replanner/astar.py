import heapq


def heuristic(graph, current, goal):

    x1, y1 = graph.get_position(current)
    x2, y2 = graph.get_position(goal)

    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current):

    path = [current]

    while current in came_from:

        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def astar(graph, start, goal):

    if not graph.has_node(start):
        raise ValueError(f"Start node '{start}' does not exist.")

    if not graph.has_node(goal):
        raise ValueError(f"Goal node '{goal}' does not exist.")

    open_list = []

    g_score = {
        node: float("inf")
        for node in graph.nodes
    }

    f_score = {
        node: float("inf")
        for node in graph.nodes
    }

    g_score[start] = 0

    f_score[start] = heuristic(
        graph,
        start,
        goal
    )

    heapq.heappush(open_list, (f_score[start], start))

    closed_set = set()

    came_from = {}

    expanded_nodes = set()
    exploration_order = []
    iterations = 0
    neighbor_evaluations = 0
    maximum_open_set_size = len(open_list)

    while open_list:

        current = heapq.heappop(open_list)[1]
        iterations += 1
        if current not in expanded_nodes:
            expanded_nodes.add(current)
            exploration_order.append(current)

        if current == goal:

            return {
                "path": reconstruct_path(
                    came_from,
                    current
                ),
                "distance": g_score[goal],
                "nodes_explored": len(expanded_nodes),
                "astar_iterations": iterations,
                "exploration_order": exploration_order,
                "closed_nodes_count": len(expanded_nodes),
                "neighbor_evaluations": neighbor_evaluations,
                "maximum_open_set_size": maximum_open_set_size,
            }

        closed_set.add(current)

        for neighbor, cost in graph.get_neighbors(current):

            neighbor_evaluations += 1

            if neighbor in closed_set:
                continue

            tentative_g = (
                g_score[current]
                + cost
            )

            if tentative_g < g_score[neighbor]:

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_score[neighbor] = (
                    tentative_g
                    + heuristic(
                        graph,
                        neighbor,
                        goal
                    )
                )

                heapq.heappush(
                    open_list,
                    (
                        f_score[neighbor],
                        neighbor
                    )
                )
                maximum_open_set_size = max(maximum_open_set_size, len(open_list))

    return None
