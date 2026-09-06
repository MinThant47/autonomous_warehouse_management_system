import networkx as nx

G = nx.Graph()

# -----------------------------
# NODES (using a 0-100 grid system)
# -----------------------------
nodes = {

    # Gate 1-4
    "Gate_1": (8, 30), "Gate_2": (8, 45),
    "Gate_3": (8, 63), "Gate_4": (8, 78), 
    
    # Parking 1 and 2
    "Parking_1": (54, 8), "Parking_2": (54, 100), "Parking_1_J": (54, 18), "Parking_2_J": (54, 90), 
   
    # Joints
    "Gate_1_J": (18, 30), "Gate_2_J": (18, 45), "Gate_3_J": (18, 63), "Gate_4_J": (18, 78),
    "Middle_1_J": (54, 45), "Middle_2_J": (54, 63),

    # Yellow Boxes Area
    "Yellow_1": (18, 8), "Yellow_1_J": (18, 18), "Yellow_2": (30, 8), "Yellow_2_J": (30, 18),
    "Yellow_3": (42, 8), "Yellow_3_J": (42, 18), "Yellow_4": (30, 28), "Yellow_5": (42, 28),

    # Purple Boxes Area
    "Purple_1": (18, 100), "Purple_1_J": (18, 90), "Purple_2": (30, 100), "Purple_2_J": (30, 90),
    "Purple_3": (42, 100), "Purple_3_J": (42, 90), "Purple_4": (30, 80), "Purple_5": (42, 80),
   
    # Green Boxes Area
    "Green_1": (66, 8), "Green_1_J": (66, 18), "Green_2": (78, 8), "Green_2_J": (78, 18),
    "Green_3": (90, 8), "Green_3_J": (90, 18), "Green_4": (66, 28), "Green_5": (78, 28),
    "Green_6": (100, 30), "Green_6_J": (90, 30), "Green_7": (100, 45), "Green_7_J": (90, 45),

    # Orange Boxes Area
    "Orange_1": (66, 100), "Orange_1_J": (66, 90), "Orange_2": (78, 100), "Orange_2_J": (78, 90),
    "Orange_3": (90, 100), "Orange_3_J": (90, 90), "Orange_4": (66, 80), "Orange_5": (78, 80),
    "Orange_6": (100, 78), "Orange_6_J": (90, 78), "Orange_7": (100, 63), "Orange_7_J": (90, 63),
}

for n, pos in nodes.items():
    G.add_node(n, pos=pos)

# -----------------------------
# EDGES (Mapping the black lines)
# -----------------------------
edges = [

    # -----------------------------
    # Gate Connections
    # -----------------------------
    ("Gate_1", "Gate_1_J"),
    ("Gate_2", "Gate_2_J"),
    ("Gate_3", "Gate_3_J"),
    ("Gate_4", "Gate_4_J"),

    # -----------------------------
    # Left Vertical Road
    # -----------------------------
    ("Yellow_1_J", "Gate_1_J"),
    ("Gate_1_J", "Gate_2_J"),
    ("Gate_2_J", "Gate_3_J"),
    ("Gate_3_J", "Gate_4_J"),
    ("Gate_4_J", "Purple_1_J"),

    # -----------------------------
    # Middle Vertical Road
    # -----------------------------
    ("Yellow_3_J", "Parking_1_J"),
    ("Parking_1_J", "Middle_1_J"),
    ("Middle_1_J", "Middle_2_J"),
    ("Middle_2_J", "Parking_2_J"),
    ("Parking_2_J", "Purple_3_J"),

    # -----------------------------
    # Right Vertical Road
    # -----------------------------
    ("Green_3_J", "Green_6_J"),
    ("Green_6_J", "Green_7_J"),
    ("Green_7_J", "Orange_7_J"),
    ("Orange_7_J", "Orange_6_J"),
    ("Orange_6_J", "Orange_3_J"),

    # -----------------------------
    # Top Horizontal Road
    # -----------------------------
    ("Yellow_1_J", "Yellow_2_J"),
    ("Yellow_2_J", "Yellow_3_J"),
    ("Yellow_3_J", "Parking_1_J"),
    ("Parking_1_J", "Green_1_J"),
    ("Green_1_J", "Green_2_J"),
    ("Green_2_J", "Green_3_J"),

    # -----------------------------
    # Middle Horizontal Road
    # -----------------------------
    ("Gate_2_J", "Middle_1_J"),
    ("Middle_1_J", "Green_7_J"),

    # -----------------------------
    # Lower Middle Horizontal Road
    # -----------------------------
    ("Gate_3_J", "Middle_2_J"),
    ("Middle_2_J", "Orange_7_J"),

    # -----------------------------
    # Bottom Horizontal Road
    # -----------------------------
    ("Purple_1_J", "Purple_2_J"),
    ("Purple_2_J", "Purple_3_J"),
    ("Purple_3_J", "Parking_2_J"),
    ("Parking_2_J", "Orange_1_J"),
    ("Orange_1_J", "Orange_2_J"),
    ("Orange_2_J", "Orange_3_J"),

    # -----------------------------
    # Parking Connections
    # -----------------------------
    ("Parking_1", "Parking_1_J"),
    ("Parking_2", "Parking_2_J"),

    # -----------------------------
    # Yellow Area Connections
    # -----------------------------
    ("Yellow_1", "Yellow_1_J"),
    ("Yellow_2", "Yellow_2_J"),
    ("Yellow_3", "Yellow_3_J"),

    ("Yellow_4", "Yellow_2_J"),
    ("Yellow_5", "Yellow_3_J"),

    # -----------------------------
    # Purple Area Connections
    # -----------------------------
    ("Purple_1", "Purple_1_J"),
    ("Purple_2", "Purple_2_J"),
    ("Purple_3", "Purple_3_J"),

    ("Purple_4", "Purple_2_J"),
    ("Purple_5", "Purple_3_J"),

    # -----------------------------
    # Green Area Connections
    # -----------------------------
    ("Green_1", "Green_1_J"),
    ("Green_2", "Green_2_J"),
    ("Green_3", "Green_3_J"),

    ("Green_4", "Green_1_J"),
    ("Green_5", "Green_2_J"),

    ("Green_6", "Green_6_J"),
    ("Green_7", "Green_7_J"),

    # -----------------------------
    # Orange Area Connections
    # -----------------------------
    ("Orange_1", "Orange_1_J"),
    ("Orange_2", "Orange_2_J"),
    ("Orange_3", "Orange_3_J"),

    ("Orange_4", "Orange_1_J"),
    ("Orange_5", "Orange_2_J"),

    ("Orange_6", "Orange_6_J"),
    ("Orange_7", "Orange_7_J"),
]

# -----------------------------
# 📏 DISTANCE FUNCTION (Manhattan)
# -----------------------------
def manhattan_dist(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

# -----------------------------
# 🔗 ADD WEIGHTED EDGES
# -----------------------------
for u, v in edges:
    G.add_edge(u, v, weight=manhattan_dist(u, v))
    
pos = nx.get_node_attributes(G, 'pos')

# plt.figure(figsize=(14, 9))

# -----------------------------
# 🎨 COLOR GROUPS
# -----------------------------
pickup_nodes = ["Gate_1", "Gate_2"]
drop_nodes = ["Gate_3", "Gate_4"]

orange_nodes = [n for n in G.nodes if n.startswith("Orange")]
yellow_nodes = [n for n in G.nodes if n.startswith("Yellow")]
green_nodes = [n for n in G.nodes if n.startswith("Green")]
purle_nodes = [n for n in G.nodes if n.startswith("Purple")]
parking_nodes = [n for n in G.nodes if n.startswith("Parking")]

# everything else = backbone / junction
node_colors = []

for node in G.nodes():
    if "_J" in node:   # 🔥 FORCE ALL *_J TO RED
        node_colors.append("slategray")
    elif node in pickup_nodes:
        node_colors.append("cyan")
    elif node in drop_nodes:
        node_colors.append("red")
    elif node in yellow_nodes:
        node_colors.append("gold")
    elif node in green_nodes:
        node_colors.append("limegreen")
    elif node in purle_nodes:
        node_colors.append("purple")
    elif node in orange_nodes:
        node_colors.append("orange")
    elif node in parking_nodes:
        node_colors.append("dodgerblue")
    else:
        node_colors.append("slategray")  # main graph

# -----------------------------
# DRAW GRAPH
# -----------------------------
# Drawing is intentionally disabled. This module supplies graph data to the
# API; importing it during backend startup must not create a Matplotlib figure
# or trigger font-cache initialization.
# -----------------------------
# 🧹 SMART LABEL SYSTEM
# -----------------------------

# show ALL labels but readable
label_pos = {}

for k, (x, y) in pos.items():
    label_pos[k] = (x + 2, y + 2)

# -----------------------------
# ✨ CLEAN FRAME
# -----------------------------
# xs = [x for x, y in pos.values()]
# ys = [y for x, y in pos.values()]

# plt.xlim(min(xs) - 5, max(xs) + 5)
# plt.ylim(min(ys) - 5, max(ys) + 5)

# plt.title("Warehouse Map Graph")
# plt.axis("equal")
# plt.grid(False)

# # -----------------------------
# # 🧾 LEGEND
# # -----------------------------
# import matplotlib.patches as mpatches

# legend = [
#     mpatches.Patch(color='cyan', label='Pickup'),
#     mpatches.Patch(color='red', label='Dropoff'),
#     mpatches.Patch(color='dodgerblue', label='Parking'),
#     mpatches.Patch(color='gold', label='Yellow Zone'),
#     mpatches.Patch(color='limegreen', label='Green Zone'),
#     mpatches.Patch(color='purple', label='Purple Zone'),
#     mpatches.Patch(color='orange', label='Orange Zone'),
#     mpatches.Patch(color='slategray', label='Junctions'),
# ]

# plt.legend(handles=legend, loc='upper left')

# plt.gca().invert_yaxis()
# # plt.savefig("new_warehouse_map.png", dpi=300, bbox_inches='tight')
# # plt.show()
# plt.close()
