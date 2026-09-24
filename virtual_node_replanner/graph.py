import json
import copy


class Graph:

    def __init__(self, filename):
        """
        Create a graph object.

        Automatically:
        1. Load JSON
        2. Validate data
        3. Build graph
        """

        self.nodes = {}
        self.original_graph = {}
        self.graph = {}

        self.load_map(filename)

    ##################################################
    # Load JSON
    ##################################################

    def load_map(self, filename):

        with open(filename, "r") as file:
            data = json.load(file)

        self.nodes = data["nodes"]

        edges = data["edges"]

        self.validate(edges)

        self.build_graph(edges)

    ##################################################
    # Validate JSON
    ##################################################

    def validate(self, edges):

        for edge in edges:

            if len(edge) != 2:
                raise ValueError(f"Invalid edge: {edge}")

            node1, node2 = edge

            if node1 == node2:
                raise ValueError(f"Node cannot connect to itself: {node1}")

            if node1 not in self.nodes:
                raise ValueError(f"{node1} does not exist.")

            if node2 not in self.nodes:
                raise ValueError(f"{node2} does not exist.")

    ##################################################
    # Manhattan Distance
    ##################################################

    def manhattan_distance(self, node1, node2):

        x1 = self.nodes[node1]["x"]
        y1 = self.nodes[node1]["y"]

        x2 = self.nodes[node2]["x"]
        y2 = self.nodes[node2]["y"]

        return abs(x1 - x2) + abs(y1 - y2)

    ##################################################
    # Build Graph
    ##################################################

    def build_graph(self, edges):

        self.original_graph = {}

        for node in self.nodes:
            self.original_graph[node] = []

        for node1, node2 in edges:

            cost = self.manhattan_distance(node1, node2)

            self.original_graph[node1].append((node2, cost))
            self.original_graph[node2].append((node1, cost))

        self.graph = copy.deepcopy(self.original_graph)

    ##################################################
    # Helper Functions
    ##################################################

    def get_neighbors(self, node):

        return self.graph[node]

    def get_position(self, node):

        return (
            self.nodes[node]["x"],
            self.nodes[node]["y"]
        )

    def has_node(self, node):

        return node in self.nodes

    ##################################################
    # Dynamic Obstacles
    ##################################################

    def remove_edge(self, node1, node2):

        self.graph[node1] = [
            edge for edge in self.graph[node1]
            if edge[0] != node2
        ]

        self.graph[node2] = [
            edge for edge in self.graph[node2]
            if edge[0] != node1
        ]

    ##################################################
    # Restore Graph
    ##################################################

    def reset_graph(self):

        self.graph = copy.deepcopy(self.original_graph)

    ##################################################
# Clone Graph
##################################################

    def clone(self):
        return copy.deepcopy(self)
    ##################################################
# Add Temporary Node
##################################################

    def add_temp_node(self, node_name, x, y):

        self.nodes[node_name] = {
            "x": x,
            "y": y
        }

        self.graph[node_name] = []

    ##################################################
# Connect Temporary Node
##################################################

    def connect_temp_node(self, node1, node2, temp):

        cost1 = self.manhattan_distance(temp, node1)
        cost2 = self.manhattan_distance(temp, node2)

        self.graph[temp].append((node1, cost1))
        self.graph[temp].append((node2, cost2))

        self.graph[node1].append((temp, cost1))
        self.graph[node2].append((temp, cost2))
    ##################################################
# Remove Temporary Node
##################################################

    def remove_temp_node(self, temp):

        if temp not in self.graph:
            return

        for neighbor, _ in list(self.graph[temp]):

            self.graph[neighbor] = [
                edge for edge in self.graph[neighbor]
                if edge[0] != temp
            ]

        del self.graph[temp]
        del self.nodes[temp]