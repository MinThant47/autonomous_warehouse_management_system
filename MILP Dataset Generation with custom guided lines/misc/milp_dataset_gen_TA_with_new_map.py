import networkx as nx
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import gurobipy as gp
from gurobipy import GRB
import numpy as np

# -----------------------------
# GLOBAL TIME SCALE 🔥
# -----------------------------
FRAMES_PER_EDGE = 10

# -----------------------------
# GRID
# -----------------------------# -----------------------------
# REAL WAREHOUSE GRAPH 🔥
# -----------------------------
from warehouse_map import G, nodes  # reuse your exact map

def heuristic(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)
# -----------------------------
# ZONE DEFINITIONS 🔥
# -----------------------------
PICKUP_NODES = ["P1", "P2", "P3"]
DROPOFF_NODES = ["D1", "D2"]

STORAGE_NODES = [
    "Blue_1", "Blue_2", "Blue_3", "Blue_4",
    "Yellow_1", "Yellow_2", "Yellow_3", "Yellow_4",
    "Green_1", "Green_2", "Green_3", "Green_4", "Green_5", "Green_6",
    "Pink_1", "Pink_2", "Pink_3", "Pink_4"
]

# -----------------------------
# ROBOTS
# -----------------------------
def generate_robots():
    return {
        "R1": {
            "pos": nodes["Charging_1"],
            "node": "Charging_1",
            "next_node": "Charging_1",
            "time_until_free": 0,
            "queue": [],
            "path": [],
            "wait": 0,
            "speed": 255
        },
        "R2": {
            "pos": nodes["Charging_2"],
            "node": "Charging_2",
            "next_node": "Charging_2",
            "time_until_free": 0,
            "queue": [],
            "path": [],
            "wait": 0,
            "speed": 255
        }
    }
# -----------------------------
# TASK
# -----------------------------
task_id = 0
    
def generate_task():
    global task_id
    task_id += 1

    # 1: Inbound, 2: Outbound, 3: Relocation
    task_type = random.choices([1, 2, 3], weights=[0.4, 0.4, 0.2])[0]

    if task_type == 1:  # Inbound
        pl = random.choice(PICKUP_NODES)
        dl = random.choice(STORAGE_NODES)

    elif task_type == 2:  # Outbound
        pl = random.choice(STORAGE_NODES)
        dl = random.choice(DROPOFF_NODES)

    else:  # Relocation
        pl = random.choice(STORAGE_NODES)
        dl = random.choice(STORAGE_NODES)

        while dl == pl:
            dl = random.choice(STORAGE_NODES)

    return {
        "id": task_id,
        "type": task_type,
        "PL": pl,
        "DL": dl,
        "pickup_time": 3,
        "dropoff_time": 3,
        "deadline": 10
    }
# -----------------------------
# TA COST (SYNCED 🔥)
# -----------------------------
def compute_total_time(robot, task):

    start = robot["next_node"]

    d1 = nx.astar_path_length(G, start, task["PL"], heuristic=heuristic, weight="weight")
    d2 = nx.astar_path_length(G, task["PL"], task["DL"], heuristic=heuristic, weight="weight")

    # 🔥 convert to frames
    d1_frames = d1 * FRAMES_PER_EDGE
    d2_frames = d2 * FRAMES_PER_EDGE

    handling_frames = int((task["pickup_time"] + task["dropoff_time"]) * FRAMES_PER_EDGE)

    return robot["time_until_free"] + d1_frames + d2_frames + handling_frames

# -----------------------------
# MILP
# -----------------------------
def solve_milp(pt, deadline):

    model = gp.Model()
    model.setParam("OutputFlag", 0)

    # K = {0,1} → two choices (e.g., 2 robots)
    # X[k] ∈ {0,1}

    K = range(2)
    X = model.addVars(K, vtype=GRB.BINARY)

    # Cmax → completion time (makespan)
    # L → lateness (deadline violation)

    Cmax = model.addVar()
    L = model.addVar()

    model.addConstr(sum(X[k] for k in K) == 1) # X[0] + X[1] = 1, only one option
    model.addConstr(sum(X[k]*pt[k] for k in K) <= Cmax)  # Cmax ≥ pt[selected_k] since X[k] = 1
    model.addConstr(sum(X[k]*pt[k] for k in K) <= deadline + L) # pt[selected_k] ≤ deadline + L
    model.addConstr(L >= 0)

    model.setObjective(Cmax + 10 * L, GRB.MINIMIZE) # Minimize Completion Time + 10 × Lateness
    model.optimize()

    # X[0].x = 1.0 or 0.0
    # X[1].x = 0.0 or 1.0
    # Why > 0.5 => X[0].x = 0.9999999 not exaclty 1
    # return 0 or 1

    for k in K:
        if X[k].x > 0.5:
            return k
    


# -----------------------------
# SMOOTH PATH (SYNCED 🔥)
# -----------------------------
def smooth_path(path):
    coords = [nodes[n] for n in path]
    smooth = []

    for i in range(len(coords)-1):
        x1, y1 = coords[i]
        x2, y2 = coords[i+1]

        dist = abs(x1 - x2) + abs(y1 - y2)
        frames = max(1, int(dist))

        for t in np.linspace(0, 1, frames):
            smooth.append((x1 + (x2-x1)*t, y1 + (y2-y1)*t))

    return smooth

# -----------------------------
# INIT
# -----------------------------
robots = generate_robots()
task_timer = 0
current_task_visual = None

# -----------------------------
# PLOT
# -----------------------------
pos = nx.get_node_attributes(G, 'pos')

pickup_nodes = ["P1", "P2", "P3"]
drop_nodes = ["D1", "D2"]

blue_nodes = [n for n in G.nodes if n.startswith("Blue")]
yellow_nodes = [n for n in G.nodes if n.startswith("Yellow")]
green_nodes = [n for n in G.nodes if n.startswith("Green")]
pink_nodes = [n for n in G.nodes if n.startswith("Pink")]
charging_nodes = [n for n in G.nodes if n.startswith("Charging")]

fig, ax = plt.subplots(figsize=(14,9))

r1_name = ax.text(0, 0, "R1", fontsize=9, color='blue', weight='bold')
r2_name = ax.text(0, 0, "R2", fontsize=9, color='green', weight='bold')

status_text = ax.text(
    0.98, 0.98, "",              # top-right corner
    transform=ax.transAxes,      # IMPORTANT (relative positioning)
    fontsize=10,
    verticalalignment='top',
    horizontalalignment='right',
    bbox=dict(facecolor='white', alpha=0.8, edgecolor='black')
)

node_colors = []

for node in G.nodes():
    if "_J" in node:   # 🔥 FORCE ALL *_J TO RED
        node_colors.append("red")
    elif node in pickup_nodes:
        node_colors.append("orange")
    elif node in drop_nodes:
        node_colors.append("cyan")
    elif node in blue_nodes:
        node_colors.append("dodgerblue")
    elif node in yellow_nodes:
        node_colors.append("gold")
    elif node in green_nodes:
        node_colors.append("limegreen")
    elif node in pink_nodes:
        node_colors.append("pink")
    elif node in charging_nodes:
        node_colors.append("purple")
    else:
        node_colors.append("red")  # main graph

# -----------------------------
# DRAW GRAPH
# -----------------------------
nx.draw_networkx_edges(G, pos, width=2, edge_color="black")
nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=50)
# -----------------------------
# 🧹 SMART LABEL SYSTEM
# -----------------------------

# show ALL labels but readable
label_pos = {}

for k, (x, y) in pos.items():
    if k.startswith("P") or k.startswith("D"):
        label_pos[k] = (x, y + 4)
    elif "J" in k:
        label_pos[k] = (x + 2, y + 2)
    elif "Blue" in k:
        label_pos[k] = (x - 2, y + 2)
    elif "Yellow" in k:
        label_pos[k] = (x + 2, y + 2)
    elif "Green" in k:
        label_pos[k] = (x + 2, y - 2)
    elif "Pink" in k:
        label_pos[k] = (x, y + 3)
    else:
        label_pos[k] = (x + 1, y + 1)

nx.draw_networkx_labels(
    G,
    label_pos,
    font_size=6,
    bbox=dict(
        facecolor="white",
        edgecolor="none",
        alpha=0.9,
        pad=0.3
    )
)
# -----------------------------
# ✨ CLEAN FRAME
# -----------------------------
xs = [x for x, y in pos.values()]
ys = [y for x, y in pos.values()]

plt.xlim(min(xs) - 5, max(xs) + 5)
plt.ylim(min(ys) - 5, max(ys) + 5)

plt.title("Warehouse Graph (Clean Visualization)")
plt.axis("equal")
plt.grid(False)

# -----------------------------
# 🧾 LEGEND
# -----------------------------
import matplotlib.patches as mpatches

legend = [
    mpatches.Patch(color='orange', label='Pickup'),
    mpatches.Patch(color='cyan', label='Dropoff'),
    mpatches.Patch(color='purple', label='Charging'),
    mpatches.Patch(color='dodgerblue', label='Blue Zone'),
    mpatches.Patch(color='gold', label='Yellow Zone'),
    mpatches.Patch(color='limegreen', label='Green Zone'),
    mpatches.Patch(color='pink', label='Pink Zone'),
    mpatches.Patch(color='red', label='Junctions'),
]

plt.legend(handles=legend, loc='upper left')


r1_dot, = ax.plot([], [], 'bo', markersize=12)
r2_dot, = ax.plot([], [], 'go', markersize=12)

# task_pick = ax.scatter([], [], c='orange', s=200)
# task_drop = ax.scatter([], [], c='blue', s=200)

# -----------------------------
# UPDATE LOOP
# -----------------------------
def update(frame):
    global task_timer, current_task_visual

    task_timer += 1

    # NEW TASK
    if task_timer > random.randint(10, 20):
        task = generate_task()
        current_task_visual = task

        r1_time = compute_total_time(robots["R1"], task)
        r2_time = compute_total_time(robots["R2"], task)

        pt = [r1_time, r2_time]
        winner_idx = solve_milp(pt, task["deadline"])
        winner = "R1" if winner_idx == 0 else "R2"

        robot = robots[winner]

        robot["time_until_free"] = pt[winner_idx]
        robot["next_node"] = task["DL"]
        robot["queue"].append(task)

        print(f"Task {task['id']} → {winner}")
        task_timer = 0

    # EXECUTION (SYNCED + STATE MACHINE)
    for name, robot in robots.items():

        # 🔥 decrement workload correctly
        if robot["time_until_free"] > 0:
            robot["time_until_free"] = max(0, robot["time_until_free"] - 1)

        # waiting
        if robot["wait"] > 0:
            robot["wait"] -= 1
            continue

        # assign task
        if "current_task" not in robot and robot["queue"]:
            robot["current_task"] = robot["queue"][0]
            robot["has_payload"] = False

        if "current_task" in robot:
            task = robot["current_task"]

            # GO TO PICKUP
            if not robot["has_payload"]:

                if robot["node"] != task["PL"]:

                    if not robot["path"]:
                        path = nx.astar_path(G, robot["node"], task["PL"], heuristic=heuristic, weight="weight")
                        robot["path"] = smooth_path(path)

                    robot["pos"] = robot["path"].pop(0)

                    if not robot["path"]:
                        robot["node"] = task["PL"]

                else:
                    robot["wait"] = int(task["pickup_time"] * FRAMES_PER_EDGE)
                    robot["has_payload"] = True

            # GO TO DROPOFF
            else:

                if robot["node"] != task["DL"]:

                    if not robot["path"]:
                        path = nx.astar_path(G, task["PL"], task["DL"], heuristic=heuristic, weight="weight")
                        robot["path"] = smooth_path(path)

                    robot["pos"] = robot["path"].pop(0)

                    if not robot["path"]:
                        robot["node"] = task["DL"]

                else:
                    robot["wait"] = int(task["dropoff_time"] * FRAMES_PER_EDGE)

                    robot["queue"].pop(0)
                    del robot["current_task"]
                    robot["has_payload"] = False

    # update visuals
    r1_dot.set_data([robots["R1"]["pos"][0]], [robots["R1"]["pos"][1]])
    r2_dot.set_data([robots["R2"]["pos"][0]], [robots["R2"]["pos"][1]])

    r1 = robots["R1"]
    r2 = robots["R2"]

    # move labels slightly above the robot
    r1_name.set_position((r1["pos"][0], r1["pos"][1] + 2))
    r2_name.set_position((r2["pos"][0], r2["pos"][1] + 2))

    if current_task_visual:
        pl = nodes[current_task_visual["PL"]]
        dl = nodes[current_task_visual["DL"]]

        # task_pick.set_offsets([pl])
        # task_drop.set_offsets([dl])
    
    print("Robot 1:", robots["R1"]["node"], "->", robots["R1"]["pos"])
    print("Robot 2:", robots["R2"]["node"], "->", robots["R2"]["pos"])

    status_lines = []

    for name, robot in robots.items():
        if "current_task" in robot:
            task = robot["current_task"]

            if not robot["has_payload"]:
                action = f"→ Going to {task['PL']}"
            else:
                action = f"→ Delivering to {task['DL']}"
        else:
            action = "Idle"

        status_lines.append(f"{name}: {action}")

    status_text.set_text("\n".join(status_lines))

    return r1_dot, r2_dot, status_text, r1_name, r2_name

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=80)
plt.show()