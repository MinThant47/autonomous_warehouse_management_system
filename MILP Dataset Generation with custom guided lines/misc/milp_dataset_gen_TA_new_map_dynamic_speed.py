import networkx as nx
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import gurobipy as gp
from gurobipy import GRB
import numpy as np

from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image
import math

import pandas as pd

dataset_rows = []
TARGET_DATASET_SIZE = 500
# -----------------------------
# GLOBAL TIME SCALE 
# -----------------------------
SIMULATION_FPS = 12.5

# 1 graph distance unit = 0.0254 meter (1 inch)
MAP_SCALE = 0.0254
# -----------------------------

# -----------------------------
# WAREHOUSE GRAPH 
# -----------------------------
from working.new_warehouse_map import G, nodes 

def heuristic(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)
# -----------------------------
# ZONE DEFINITIONS 
# -----------------------------
PICKUP_NODES = ["Gate_1", "Gate_2"]
DROPOFF_NODES = ["Gate_3", "Gate_4"]

STORAGE_NODES = [
    "Yellow_1", "Yellow_2", "Yellow_3", "Yellow_4", "Yellow_5",
    "Purple_1", "Purple_2", "Purple_3", "Purple_4", "Purple_5",
    "Green_1", "Green_2", "Green_3", "Green_4", "Green_5", "Green_6", "Green_7",
    "Orange_1", "Orange_2", "Orange_3", "Orange_4", "Orange_5", "Orange_6", "Orange_7",
]

PARKING_STATIONS = ["Parking_1", "Parking_2"]

# Track which robot owns which station
parking_occupancy = {
    "Parking_1": None,
    "Parking_2": None
}

# -----------------------------
# ROBOTS
# -----------------------------
def generate_robots():
    return {
        "R1": {
            "pos": nodes["Parking_1"],
            "node": "Parking_1",
            "next_node": "Parking_1",
            # PURE PHYSICAL SECONDS
            "real_time_until_free": 0.0,
            # VISUAL FRAME COUNTDOWN
            "time_until_free": 0,
            "queue": [],
            "path": [],
            "wait": 0,
            "angle": 0,
            "speed": 0.5
        },
        "R2": {
            "pos": nodes["Parking_2"],
            "node": "Parking_2",
            "next_node": "Parking_2",
            # PURE PHYSICAL SECONDS
            "real_time_until_free": 0.0,
            # VISUAL FRAME COUNTDOWN
            "time_until_free": 0,
            "queue": [],
            "path": [],
            "wait": 0,
            "angle": 0,
            "speed": 0.5
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
        "deadline": random.choice([10, 20, 30])
    }
# -----------------------------
# TA COST (SYNCED)
# -----------------------------
# -----------------------------
# REAL PHYSICAL TIME
# -----------------------------
def compute_task_time_seconds(robot, task):

    start = robot["next_node"]

    # distance from Next Node to Pickup
    d1 = nx.astar_path_length(
        G,
        start,
        task["PL"],
        heuristic=heuristic,
        weight="weight"
    )

    # distance from Pickup to Dropoff
    d2 = nx.astar_path_length(
        G,
        task["PL"],
        task["DL"],
        heuristic=heuristic,
        weight="weight"
    )

    # convert graph units -> meters
    d1_m = d1 * MAP_SCALE
    d2_m = d2 * MAP_SCALE

    # travel times
    t1 = d1_m / robot["speed"]
    t2 = d2_m / robot["speed"]

    # ONLY this task duration
    task_duration = (
        t1 +
        t2 +
        task["pickup_time"] +
        task["dropoff_time"]
    )

    return task_duration
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
# VISUAL ANIMATION PATH
# -----------------------------
def smooth_path(path, robot_speed, has_payload):

    # Change from ["P2", "Blue_2", "P1"] to [(90,0), (100,80), (80,30)]
    coords = [nodes[n] for n in path] 
    smooth = []

    actual_speed = robot_speed

    for i in range(len(coords)-1):

        x1, y1 = coords[i]
        x2, y2 = coords[i+1]

        # graph distance
        dist = abs(x1 - x2) + abs(y1 - y2)

        # convert to meters
        dist_m = dist * MAP_SCALE

        # physical travel time
        travel_time_s = (
            dist_m / actual_speed
        )

        # convert seconds → animation frames
        # “How many animation steps should this movement take?”
        # minimum 1 movement step if time * simulation result in sth like 0.25 and int round it to 0

        frames = max(
            1,
            int(travel_time_s * SIMULATION_FPS)
        )

        # for eg. robot needs 2.4 REAL seconds to travel
        # simulation runs at 12.5 frames/sec

        # Then:

        # frames = 2.4 × 12.5 = 30

        for t in np.linspace(0, 1, frames):

            smooth.append((
                x1 + (x2-x1)*t,
                y1 + (y2-y1)*t
            ))

    return smooth
# -----------------------------
# INIT
# -----------------------------
robots = generate_robots()
task_counter = 0
current_task_visual = None

# -----------------------------
# PLOT
# -----------------------------
pos = nx.get_node_attributes(G, 'pos')

pickup_nodes = ["Gate_1", "Gate_2"]
drop_nodes = ["Gate_3", "Gate_4"]

orange_nodes = [n for n in G.nodes if n.startswith("Orange")]
yellow_nodes = [n for n in G.nodes if n.startswith("Yellow")]
green_nodes = [n for n in G.nodes if n.startswith("Green")]
purle_nodes = [n for n in G.nodes if n.startswith("Purple")]
parking_nodes = [n for n in G.nodes if n.startswith("Parking")]

fig, ax = plt.subplots(figsize=(14,9))

# -----------------------------
# ROBOT IMAGES
# -----------------------------

robot_images = {
    "R1": Image.open("robot1.png").convert("RGBA"),
    "R2": Image.open("robot2.png").convert("RGBA")
}


r1_status_text = ax.text(
    0.98,
    0.98,
    "",
    transform=ax.transAxes,
    fontsize=10,
    verticalalignment='top',
    horizontalalignment='right',
    color='red',
    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
)

r2_status_text = ax.text(
    0.98,
    0.93,
    "",
    transform=ax.transAxes,
    fontsize=10,
    verticalalignment='top',
    horizontalalignment='right',
    color='dodgerblue',
    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')
)

queue_text = ax.text(
    0.85,
    0.85,
    "",
    transform=ax.transAxes,
    fontsize=9,
    verticalalignment='top',
    horizontalalignment='left',
    bbox=dict(
        facecolor='white',
        alpha=0.9,
        edgecolor='black'
    )
)

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
nx.draw_networkx_edges(G, pos, width=2, edge_color="black")
nx.draw_networkx_nodes(G, pos, node_color=node_colors, node_size=50)

# The positions of color match by index.

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

# Extract all x/y coordinates

# pos = {
#     "A": (10, 20),
#     "B": (30, 15),
#     "C": (50, 40)
# }

xs = [x for x, y in pos.values()] #xs = [10, 30, 50]
ys = [y for x, y in pos.values()] #ys = [20, 15, 40]

plt.xlim(min(xs) - 5, max(xs) + 5) #plt.xlim(5, 55)
plt.ylim(min(ys) - 5, max(ys) + 5) #plt.ylim(10, 45)

plt.title("Warehouse Graph")
plt.axis("equal")
plt.grid(False)

# -----------------------------
# 🧾 LEGEND
# -----------------------------
import matplotlib.patches as mpatches

legend = [
    mpatches.Patch(color='cyan', label='Pickup'),
    mpatches.Patch(color='red', label='Dropoff'),
    mpatches.Patch(color='dodgerblue', label='Parking'),
    mpatches.Patch(color='gold', label='Yellow Zone'),
    mpatches.Patch(color='limegreen', label='Green Zone'),
    mpatches.Patch(color='purple', label='Purple Zone'),
    mpatches.Patch(color='orange', label='Orange Zone'),
    mpatches.Patch(color='slategray', label='Junctions'),
]

plt.legend(handles=legend, loc='upper left')


# -----------------------------
# ROBOT VISUALS
# -----------------------------
r1_imgbox = OffsetImage(np.zeros((10,10,4)), zoom=0.03)
r2_imgbox = OffsetImage(np.zeros((10,10,4)), zoom=0.03)

r1_visual = AnnotationBbox(r1_imgbox, (0,0), frameon=False)
r2_visual = AnnotationBbox(r2_imgbox, (0,0), frameon=False)

ax.add_artist(r1_visual)
ax.add_artist(r2_visual)


def rotated_robot_image(robot_name, angle):

    rotated = robot_images[robot_name].rotate(
        -angle,
        expand=False
    )

    return np.array(rotated)

# To change route from charging to new task
# Convert continuous position -> nearest graph node
# When there is a new task, robot could be going to charging nodes; it just detects the nearest node so that it can replan the route
def nearest_graph_node(position):

    best_node = None
    best_dist = float("inf")

    px, py = position

    for node_name, (nx_, ny_) in nodes.items():

        dist = math.sqrt(
            (px - nx_)**2 +
            (py - ny_)**2
        )

        if dist < best_dist:
            best_dist = dist
            best_node = node_name

    return best_node

# -----------------------------
# UPDATE LOOP
# -----------------------------
def update(frame):
    global task_counter, current_task_visual

    task_counter += 1

    # NEW TASK
    if task_counter > random.randint(80, 100):
        task = generate_task()
        current_task_visual = task

        r1_new_task_time = compute_task_time_seconds(
            robots["R1"],
            task
        )

        r2_new_task_time = compute_task_time_seconds(
            robots["R2"],
            task
        )

        # ---------------------------------
        # PREDICTED COMPLETION TIMES for both robots
        # ---------------------------------

        r1_time = (
            robots["R1"]["real_time_until_free"]
            + r1_new_task_time
        )

        r2_time = (
            robots["R2"]["real_time_until_free"]
            + r2_new_task_time
        )

        # MILP INPUT
        pt = [r1_time, r2_time]

        winner_idx = solve_milp(
            pt,
            task["deadline"]
        )

        winner = "R1" if winner_idx == 0 else "R2"
        robot = robots[winner]

        # -----------------------------
        # INTERRUPT CHARGING MOVEMENT
        # -----------------------------

        # if robot was idle-moving to charging
        if ( "current_task" not in robot and robot["path"]):

            # stop old charging path
            robot["path"] = []

            # update logical node
            robot["node"] = nearest_graph_node(
                robot["pos"]
            )

            # release charging reservation
            for station, occupant in parking_occupancy.items():

                if occupant == winner:
                    parking_occupancy[station] = None

        # -----------------------------
        # ASSIGN TASK
        # -----------------------------
        # PURE PHYSICAL SECONDS
        
        if winner == "R1":

            robots["R1"]["real_time_until_free"] += (
                r1_new_task_time
            )

        else:

            robots["R2"]["real_time_until_free"] += (
                r2_new_task_time
            )

        # VISUAL FRAME COUNTDOWN
        robot["time_until_free"] = math.ceil(
            pt[winner_idx] * SIMULATION_FPS
        )
        robot["next_node"] = task["DL"]
        robot["queue"].append(task)

        # -----------------------------
        # ML DATASET SAMPLE
        # -----------------------------

        r1_nx, r1_ny = nodes[
            robots["R1"]["next_node"]
        ]

        r2_nx, r2_ny = nodes[
            robots["R2"]["next_node"]
        ]

        pl_x, pl_y = nodes[task["PL"]]
        dl_x, dl_y = nodes[task["DL"]]

        # -----------------------------
        # DISTANCE FEATURES
        # -----------------------------

        r1_dist_to_pickup = nx.astar_path_length(
            G,
            robots["R1"]["next_node"],
            task["PL"],
            heuristic=heuristic,
            weight="weight"
        )

        r2_dist_to_pickup = nx.astar_path_length(
            G,
            robots["R2"]["next_node"],
            task["PL"],
            heuristic=heuristic,
            weight="weight"
        )

        task_dist_to_dropoff = nx.astar_path_length(
            G,
            task["PL"],
            task["DL"],
            heuristic=heuristic,
            weight="weight"
        )

        X_features = {

            "R1_next_x": round(r1_nx * MAP_SCALE, 4),
            "R1_next_y": round(r1_ny * MAP_SCALE, 4),
            "R1_tuf": round(
                robots["R1"]["real_time_until_free"],
                2
            ),

            "R2_next_x": round(r2_nx * MAP_SCALE, 4),
            "R2_next_y": round(r2_ny * MAP_SCALE, 4),
            "R2_tuf": round(
                robots["R2"]["real_time_until_free"],
                2
            ),

            "R1_queue_length": len(robots["R1"]["queue"]),
            "R2_queue_length": len(robots["R2"]["queue"]),

            "tuf_difference": round(robots["R1"]["real_time_until_free"] - robots["R2"]["real_time_until_free"], 2),

            "Task_PL_x": round(pl_x * MAP_SCALE, 4),
            "Task_PL_y": round(pl_y * MAP_SCALE, 4),

            "Task_DL_x": round(dl_x * MAP_SCALE, 4),
            "Task_DL_y": round(dl_y * MAP_SCALE, 4),

            "R1_dist_to_pickup": round(r1_dist_to_pickup * MAP_SCALE, 4),
            "R2_dist_to_pickup": round(r2_dist_to_pickup * MAP_SCALE, 4),
            "task_dist_to_dropoff": round(task_dist_to_dropoff * MAP_SCALE, 4),

            "task_type": task["type"],

            "Deadline": task["deadline"]
        }

        y_output = winner_idx

        # -----------------------------
        # SAVE DATASET ROW
        # -----------------------------

        dataset_row = X_features.copy()
        dataset_row["winner"] = y_output
        dataset_rows.append(dataset_row)

        # -----------------------------
        # STOP WHEN DATASET FULL
        # -----------------------------

        if len(dataset_rows) >= TARGET_DATASET_SIZE:

            df = pd.DataFrame(dataset_rows)

            df.to_csv(
                f"{TARGET_DATASET_SIZE} warehouse_task_allocation_xgboost_dataset.csv",
                index=False
            )

            print("\n==============================")
            print("DATASET GENERATION COMPLETE")
            print(f"Saved {len(df)} rows")
            print("warehouse_task_allocation_xgboost_dataset.csv")
            print("==============================")

            plt.close()

        print(f"NEW TASK #{task['id']}")

        print(
            f"Pickup : {task['PL']} "
            f"-> Dropoff : {task['DL']}"
        )

        print(f"Assigned Robot : {winner}")

        print("==============================\n")
        task_counter = 0

    # EXECUTION (SYNCED + STATE MACHINE)
    for name, robot in robots.items():

        # 🔥 decrement workload correctly
        if robot["time_until_free"] > 0:

            # VISUAL FRAMES
            robot["time_until_free"] = max(
                0,
                robot["time_until_free"] - 1
            )

            # REAL PHYSICAL TIME
            dt = 1.0 / SIMULATION_FPS

            robot["real_time_until_free"] = max(
                0.0,
                robot["real_time_until_free"] - dt
            )

        # waiting
        if robot["wait"] > 0:
            robot["wait"] -= 1
            continue

        # assign task
        if "current_task" not in robot and robot["queue"]:
            robot["current_task"] = robot["queue"][0]
            robot["has_payload"] = False


            # 🔥 release charging station
            for station, occupant in parking_occupancy.items():
                if occupant == name:
                    parking_occupancy[station] = None


        # 🔋 IDLE → GO TO CHARGING
        if "current_task" not in robot:

            # if not already at charging station
            if robot["node"] not in PARKING_STATIONS:

                if not robot["path"]:

                    # available stations
                    # free charger allowed OR already reserved by same robot allowed
                    available = [
                        s for s, occ in parking_occupancy.items()
                        if occ is None or occ == name
                    ]

                    if available:
                        best_station = None
                        min_dist = float('inf')

                        for station in available:
                            dist = nx.astar_path_length(
                                G, robot["node"], station,
                                heuristic=heuristic, weight="weight"
                            )

                            if dist < min_dist:
                                min_dist = dist
                                best_station = station

                        if best_station:
                            parking_occupancy[best_station] = name

                            path = nx.astar_path(
                                G, robot["node"], best_station,
                                heuristic=heuristic, weight="weight"
                            )

                            robot["path"] = smooth_path(
                                path, robot["speed"], False
                            )

                # move
                if robot["path"]:
                    robot["pos"] = robot["path"].pop(0)

                    if not robot["path"]:
                        robot["node"] = next(
                            (s for s, occ in parking_occupancy.items() if occ == name),
                            robot["node"]
                        )

            continue

        if "current_task" in robot:
            task = robot["current_task"]

            # GO TO PICKUP
            if not robot["has_payload"]:

                if robot["node"] != task["PL"]:

                    if not robot["path"]:
                        path = nx.astar_path(G, robot["node"], task["PL"], heuristic=heuristic, weight="weight")
                        robot["path"] = smooth_path(path, robot["speed"], False)

                    robot["pos"] = robot["path"].pop(0)

                    if not robot["path"]:
                        robot["node"] = task["PL"]

                else:
                    robot["wait"] = int(task["pickup_time"] * SIMULATION_FPS)
                    robot["has_payload"] = True

            # GO TO DROPOFF
            else:

                if robot["node"] != task["DL"]:

                    if not robot["path"]:
                        path = nx.astar_path(G, task["PL"], task["DL"], heuristic=heuristic, weight="weight")
                        robot["path"] = smooth_path(path, robot["speed"], True)

                    robot["pos"] = robot["path"].pop(0)

                    if not robot["path"]:
                        robot["node"] = task["DL"]

                else:
                    robot["wait"] = int(task["dropoff_time"] * SIMULATION_FPS)
                    robot["queue"].pop(0)
                    del robot["current_task"]
                    robot["has_payload"] = False

    # -----------------------------
    # ROTATING ROBOTS
    # -----------------------------

    for robot_name, visual, imgbox in [
        ("R1", r1_visual, r1_imgbox),
        ("R2", r2_visual, r2_imgbox)
    ]:

        robot = robots[robot_name]

        x, y = robot["pos"]

        # keep previous angle
        angle = robot["angle"]

        # update only if movement is meaningful
        if robot["path"]:

            next_x, next_y = robot["path"][0]

            dx = next_x - x
            dy = next_y - y

            movement = math.sqrt(dx*dx + dy*dy)

            # ignore tiny jitter
            if movement > 0.05:

                angle = math.degrees(math.atan2(dy, dx)) + 90

                # save stable angle
                robot["angle"] = angle

        rotated_img = rotated_robot_image(robot_name, angle)

        imgbox.set_data(rotated_img)

        visual.xybox = (x, y)
        visual.xy = (x, y)

    for name, robot in robots.items():

        if "current_task" in robot:

            task = robot["current_task"]

            if not robot["has_payload"]:
                action = f"→ Going to {task['PL']}"
            else:
                action = f"→ Delivering to {task['DL']}"

        else:

            if robot["path"]:

                target_station = None

                for station, occupant in parking_occupancy.items():

                    if occupant == name:
                        target_station = station
                        break

                if target_station:
                    action = f"Going to {target_station}"
                else:
                    action = "Returning"

            else:
                action = "Idle"

        if name == "R1":
            r1_status_text.set_text(
                f"● {name}: {action}"
            )

        else:
            r2_status_text.set_text(
                f"● {name}: {action}"
            )
    
    queue_display = []

    for robot_name, robot in robots.items():

        queue_display.append(
            f"{robot_name} Queue:"
        )

        if robot["queue"]:

            for i, t in enumerate(robot["queue"]):

                queue_display.append(
                    f"  {i+1}. "
                    f"{t['PL']} → {t['DL']}"
                )

        else:
            queue_display.append("  Empty")

        queue_display.append("")

    queue_text.set_text(
        "\n".join(queue_display)
    )

    return r1_visual, r2_visual, r1_status_text, r2_status_text, queue_text

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=80)
plt.gca().invert_yaxis()
plt.show()