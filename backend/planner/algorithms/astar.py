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

    heapq.heappush(open_list, (0, start))

    closed_set = set()

    came_from = {}

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

    while open_list:

        current = heapq.heappop(open_list)[1]

        if current == goal:

            return {
                "path": reconstruct_path(
                    came_from,
                    current
                ),
                "distance": g_score[goal]
            }

        closed_set.add(current)

        for neighbor, cost in graph.get_neighbors(current):

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

    return None