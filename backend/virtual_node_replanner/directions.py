class DirectionGenerator:
    """
    Converts an A* path into robot movement commands.
    """

    def __init__(self, graph):
        self.graph = graph

    ####################################################
    # Vector
    ####################################################

    def get_vector(self, node1, node2):

        x1, y1 = self.graph.get_position(node1)
        x2, y2 = self.graph.get_position(node2)

        return (
            x2 - x1,
            y2 - y1
        )
    def normalize(self, vector):

        x, y = vector

        if x > 0:
            x = 1
        elif x < 0:
            x = -1
        else:
            x = 0
        if y > 0:
            y = 1
        elif y < 0:
            y = -1
        else:
            y = 0 

        return (x, y)
    ####################################################
    # Determine Turn
    ####################################################

    def get_action(self, previous, current, next_node):

        incoming = self.normalize(
            self.get_vector(previous, current)
        )

        outgoing = self.normalize(
            self.get_vector(current, next_node)
        )

        # Same direction
        if incoming == outgoing:
            return "FORWARD"

        # Reverse
        if incoming == (-outgoing[0], -outgoing[1]):
            return "BACK"

        # Cross Product
        cross = (
            incoming[0] * outgoing[1]
            - incoming[1] * outgoing[0]
        )

        if cross > 0:
            return "Right"

        if cross < 0:
            return "Left"

        return "UNKNOWN"

    ####################################################
    # Generate Direction List
    ####################################################

    def generate(self, path):

        if len(path) == 0:
            return []

        if len(path) == 1:
            return [
                {
                    "node": path[0],
                    "action": "STOP"
                }
            ]

        directions = []

        directions.append({
            "node": path[0],
            "action": "START"
        })

        for i in range(1, len(path)-1):

            action = self.get_action(
                path[i-1],
                path[i],
                path[i+1]
            )

            directions.append({
                "node": path[i],
                "action": action
            })

        directions.append({
            "node": path[-1],
            "action": "STOP"
        })

        return directions