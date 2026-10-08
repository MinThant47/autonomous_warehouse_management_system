import networkx as nx
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import json
import joblib
import pandas as pd

from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image

import math
import time

from task_database import (
    initialize_database,
    insert_task,
    update_task_status,
    update_remaining_time,
    update_actual_completion_time,
    calculate_robot_rtuf,
    get_robot_tasks,
    print_all_tasks
)
import pandas as pd

# -----------------------------
# GLOBAL TIME SCALE 
# -----------------------------
SIMULATION_FPS = 12.5
simulation_done = False

# 1 graph distance unit = 0.0254 meter (1 inch)
MAP_SCALE = 0.025333
# -----------------------------
file_name = "new_10"
random_seed = 42

initialize_database()
with open(f"Tasks/{file_name}.json", "r") as f:
    scenario_tasks = json.load(f)

scenario_index = 0

MIN_INTERVAL_SECONDS = 1
MAX_INTERVAL_SECONDS = 10

random.seed(random_seed)

ARRIVAL_PATTERN = [
    random.randint(
        MIN_INTERVAL_SECONDS,
        MAX_INTERVAL_SECONDS
    )
    for _ in range(10000)      # much larger than your task count
]

arrival_index = 0

BATCH_INTERVAL_FRAMES = int(
    ARRIVAL_PATTERN[arrival_index] * SIMULATION_FPS
)

classifier = joblib.load(
    "Classifiers/xgb_classifier_deadline.pkl"
)

evaluation_metrics = {
    "makespan": 0.0,
    "computation_time": 0.0,
    "cumulative_penalty": 0.0,
    "task_success_rate": 0.0,
}

evaluation_counters = {
    "total_tasks_generated": 0,
    "completed_tasks": 0,
    "successful_tasks": 0,
}

# -----------------------------
# WAREHOUSE GRAPH 
# -----------------------------
from new_warehouse_map import G, nodes 

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
            "logical_path": [],
            "visual_path": [],
            "wait": 0,
            "angle": 0,
            "speed": 0.25333
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
            "logical_path": [],
            "visual_path": [],
            "wait": 0,
            "angle": 0,
            "speed": 0.25333
        }
    }

def get_classifier_feature_names(model):

    if hasattr(model, "feature_names_in_"):
        return list(model.feature_names_in_)

    booster = model.get_booster()

    if booster.feature_names:
        return list(booster.feature_names)

    raise RuntimeError("Classifier feature names not found.")

CLASSIFIER_COLUMNS = get_classifier_feature_names(classifier)

def build_classifier_features(task, robot_state):

    r1_next = robot_state["R1"]["next_node"]
    r2_next = robot_state["R2"]["next_node"]

    r1x, r1y = nodes[r1_next]
    r2x, r2y = nodes[r2_next]

    plx, ply = nodes[task["PL"]]
    dlx, dly = nodes[task["DL"]]

    r1_dist = (
        nx.astar_path_length(
            G,
            r1_next,
            task["PL"],
            heuristic=heuristic,
            weight="weight"
        )
        * MAP_SCALE
    )

    r2_dist = (
        nx.astar_path_length(
            G,
            r2_next,
            task["PL"],
            heuristic=heuristic,
            weight="weight"
        )
        * MAP_SCALE
    )

    task_dist = (
        nx.astar_path_length(
            G,
            task["PL"],
            task["DL"],
            heuristic=heuristic,
            weight="weight"
        )
        * MAP_SCALE
    )

    row = {

        "r1_next_x": round(r1x * MAP_SCALE, 4),
        "r1_next_y": round(r1y * MAP_SCALE, 4),
        "r1_tuf": round(robot_state["R1"]["real_time_until_free"], 2),

        "r2_next_x": round(r2x * MAP_SCALE, 4),
        "r2_next_y": round(r2y * MAP_SCALE, 4),
        "r2_tuf": round(robot_state["R2"]["real_time_until_free"], 2),

        "r1_queue_length": robot_state["R1"]["queue_length"],
        "r2_queue_length": robot_state["R2"]["queue_length"],

        "tuf_difference": round(
            robot_state["R1"]["real_time_until_free"]
            - robot_state["R2"]["real_time_until_free"],
            2
        ),

        "task_pl_x": round(plx * MAP_SCALE, 4),
        "task_pl_y": round(ply * MAP_SCALE, 4),
        "task_dl_x": round(dlx * MAP_SCALE, 4),
        "task_dl_y": round(dly * MAP_SCALE, 4),

        "r1_dist_to_pickup": round(r1_dist, 4),
        "r2_dist_to_pickup": round(r2_dist, 4),

        "task_dist_to_dropoff": round(task_dist, 4),

        "task_type": task["type"],
        "deadline": task["deadline"],
    }

    df = pd.DataFrame([row])
    return df[CLASSIFIER_COLUMNS]

def apply_xgboost_fifo(
    task,
    classifier
):

    robot_state = snapshot_robot_state()

    print("\n==============================")
    print("XGBOOST FIFO ASSIGNMENT")
    print("==============================")


    features = build_classifier_features(
        task,
        robot_state
    )

    prediction_start = time.perf_counter()

    prediction = classifier.predict(features)[0]

    evaluation_metrics["computation_time"] += (
        time.perf_counter() - prediction_start
    )

    robot_name = "R1" if prediction == 0 else "R2"

    robot = robots[robot_name]

    interrupt_parking_movement(robot_name)

    task_duration = compute_task_time_seconds_from_node(
        robot["next_node"],
        robot,
        task
    )

    insert_task(
        task_id=task["id"],
        task_type=task["type"],
        PL=task["PL"],
        DL=task["DL"],
        assigned_robot=robot_name,
        predicted_duration=task_duration
    )

    evaluation_counters["total_tasks_generated"] += 1

    task["assigned_robot"] = robot_name

    robot["queue"].append(task)

    update_current_task_remaining_time(robot_name)

    robot["time_until_free"] = math.ceil(
        robot["real_time_until_free"]
        * SIMULATION_FPS
    )
    
    if robot["queue"]:
        robot["next_node"] = robot["queue"][-1]["DL"]
    else:
        robot["next_node"] = robot["node"]

    print(
        f"T{task['id']:2d}"
        f" -> {robot_name}"
        f" | Queue={len(robot['queue'])}"
        f" | RTUF={robot['real_time_until_free']:.2f}"
    )

    robot_state = snapshot_robot_state()

    print("==============================\n")
    
# -----------------------------
# TA COST (SYNCED)
# -----------------------------
# -----------------------------
# REAL PHYSICAL TIME
# -----------------------------

def compute_task_time_seconds_from_node(start_node, robot, task):

    d1 = nx.astar_path_length(
        G,
        start_node,
        task["PL"],
        heuristic=heuristic,
        weight="weight"
    )

    d2 = nx.astar_path_length(
        G,
        task["PL"],
        task["DL"],
        heuristic=heuristic,
        weight="weight"
    )

    return (
        ((d1 + d2) * MAP_SCALE)
        / robot["speed"]
        + task["pickup_time"]
        + task["dropoff_time"]
    )


def snapshot_robot_state():

    return {
        robot_name: {
            "next_node": robot["next_node"],
            "real_time_until_free": robot["real_time_until_free"],
            "queue_length": len(get_robot_tasks(robot_name))
        }
        for robot_name, robot in robots.items()
    }


def encode_robot_label(robot_name):

    return {
        "R1": 0,
        "R2": 1
    }[robot_name]


def build_batch_dataset_row(
    batch_id,
    batch_position,
    batch_size,
    task,
    robot_state,
    assigned_robot,
    sequence_rank
):

    r1_nx, r1_ny = nodes[robot_state["R1"]["next_node"]]
    r2_nx, r2_ny = nodes[robot_state["R2"]["next_node"]]
    pl_x, pl_y = nodes[task["PL"]]
    dl_x, dl_y = nodes[task["DL"]]

    r1_dist_to_pickup = nx.astar_path_length(
        G,
        robot_state["R1"]["next_node"],
        task["PL"],
        heuristic=heuristic,
        weight="weight"
    )

    r2_dist_to_pickup = nx.astar_path_length(
        G,
        robot_state["R2"]["next_node"],
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

    r1_tuf = robot_state["R1"]["real_time_until_free"]
    r2_tuf = robot_state["R2"]["real_time_until_free"]

    return {
        "batch_id": f"B{batch_id:05d}",
        "group_id": f"B{batch_id:05d}_{assigned_robot}",
        "task_id": task["id"],
        "batch_position": batch_position,
        "batch_size": batch_size,

        "r1_next_x": round(r1_nx * MAP_SCALE, 4),
        "r1_next_y": round(r1_ny * MAP_SCALE, 4),
        "r1_tuf": round(r1_tuf, 2),

        "r2_next_x": round(r2_nx * MAP_SCALE, 4),
        "r2_next_y": round(r2_ny * MAP_SCALE, 4),
        "r2_tuf": round(r2_tuf, 2),

        "r1_queue_length": robot_state["R1"]["queue_length"],
        "r2_queue_length": robot_state["R2"]["queue_length"],
        "tuf_difference": round(r1_tuf - r2_tuf, 2),

        "task_pl_x": round(pl_x * MAP_SCALE, 4),
        "task_pl_y": round(pl_y * MAP_SCALE, 4),
        "task_dl_x": round(dl_x * MAP_SCALE, 4),
        "task_dl_y": round(dl_y * MAP_SCALE, 4),

        "r1_dist_to_pickup": round(r1_dist_to_pickup * MAP_SCALE, 4),
        "r2_dist_to_pickup": round(r2_dist_to_pickup * MAP_SCALE, 4),
        "task_dist_to_dropoff": round(task_dist_to_dropoff * MAP_SCALE, 4),

        "task_type": task["type"],
        "deadline": task["deadline"],
        "assigned_robot": encode_robot_label(assigned_robot),
        "sequence_rank": sequence_rank
    }

def interrupt_parking_movement(robot_name):

    robot = robots[robot_name]

    if "current_task" in robot or not robot["logical_path"]:
        return

    robot["logical_path"] = []
    robot["visual_path"] = []
    robot["node"] = nearest_graph_node(robot["pos"])

    for station, occupant in parking_occupancy.items():

        if occupant == robot_name:
            parking_occupancy[station] = None

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


def build_visual_path(robot):

    if len(robot["logical_path"]) < 2:
        robot["visual_path"] = []
        return

    robot["visual_path"] = smooth_path(
        robot["logical_path"],
        robot["speed"],
        robot.get("has_payload", False)
    )
# -----------------------------
# INIT
# -----------------------------
robots = generate_robots()
task_counter = 0
batch_counter = 0
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

def export_metrics(metrics):

    with open(
        f"Metrics/1-10/{file_name}_random_seed_{random_seed}_xgboost_fifo_metrics.json",
        "w"
    ) as f:

        json.dump(
            metrics,
            f,
            indent=4
        )

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


def update_current_task_remaining_time(robot_name):

    robot = robots[robot_name]

    if not robot["queue"]:
        return

    next_start_node = robot["node"]

    for queue_index, task in enumerate(robot["queue"]):

        is_current_task = (
            queue_index == 0
            and "current_task" in robot
            and task["id"] == robot["current_task"]["id"]
        )

        if is_current_task and robot["has_payload"]:

            d = nx.astar_path_length(
                G,
                robot["node"],
                task["DL"],
                heuristic=heuristic,
                weight="weight"
            )

            remaining_time = (
                (d * MAP_SCALE)
                / robot["speed"]
                + task["dropoff_time"]
            )

        else:

            remaining_time = compute_task_time_seconds_from_node(
                next_start_node,
                robot,
                task
            )

        update_remaining_time(
            task["id"],
            remaining_time
        )

        next_start_node = task["DL"]

    robot["real_time_until_free"] = (calculate_robot_rtuf(robot_name))

def simulation_finished():

    if scenario_index < len(scenario_tasks):
        return False

    for robot in robots.values():

        if robot["queue"]:
            return False

        if "current_task" in robot:
            return False

    return True

def finalize_metrics(frame):

    evaluation_metrics["makespan"] = round(
        frame / SIMULATION_FPS,
        2
    )

    evaluation_metrics["task_success_rate"] = (
        round(
            evaluation_counters["successful_tasks"]
            /
            evaluation_counters["total_tasks_generated"]
            * 100,
            2
        )
        if evaluation_counters["total_tasks_generated"]
        else 0
    )

    evaluation_metrics["cumulative_penalty"] = round(
        evaluation_metrics["cumulative_penalty"],
        2
    )

    evaluation_metrics["computation_time"] = round(
        evaluation_metrics["computation_time"],
        6
    )

# -----------------------------
# UPDATE LOOP
# -----------------------------
def update(frame):
    global task_counter, batch_counter, current_task_visual, scenario_index
    global BATCH_INTERVAL_FRAMES
    global arrival_index

    task_counter += 1

    if task_counter >= BATCH_INTERVAL_FRAMES and scenario_index < len(scenario_tasks):
        task_counter = 0

        arrival_seconds = ARRIVAL_PATTERN[arrival_index]
        arrival_index += 1

        print(f"Arrival Interval = {arrival_seconds} s")

        BATCH_INTERVAL_FRAMES = int(
            arrival_seconds * SIMULATION_FPS
        )

    # DISPATCH WINDOW
    # if frame % TASK_INTERVAL_FRAMES == 0 and scenario_index < len(scenario_tasks):

        new_task = scenario_tasks[scenario_index]
        new_task["created_sim_time"] = (
            frame / SIMULATION_FPS
        )
        scenario_index += 1

        current_task_visual = new_task

        apply_xgboost_fifo(
            new_task,
            classifier
        )

        print(f"Task {new_task['id']} arrived.")

        if scenario_index >= len(scenario_tasks):
            print("Scenario completed.")

            print("==============================\n")

    # EXECUTION (SYNCED + STATE MACHINE)
    for name, robot in robots.items():

        # 🔥 decrement workload correctly
        if robot["time_until_free"] > 0:

            # VISUAL FRAMES
            robot["time_until_free"] = max(
                0,
                robot["time_until_free"] - 1
            )

        # waiting
        if robot["wait"] > 0:
            robot["wait"] -= 1
            continue

        # assign task
        if "current_task" not in robot and robot["queue"]:
            robot["current_task"] = robot["queue"][0]
            robot["current_task"]["actual_start_time"] = (
                frame / SIMULATION_FPS
            )
            robot["has_payload"] = False

            update_task_status(
                robot["current_task"]["id"],
                "IN_PROGRESS"
            )


            # 🔥 release charging station
            for station, occupant in parking_occupancy.items():
                if occupant == name:
                    parking_occupancy[station] = None


        # 🔋 IDLE → GO TO CHARGING
        if "current_task" not in robot:

            # if not already at charging station
            if robot["node"] not in PARKING_STATIONS:

                if len(robot["logical_path"]) <= 1:

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

                        # here
                        if best_station:

                            parking_occupancy[best_station] = name

                            robot["logical_path"] = nx.astar_path(
                                G,
                                robot["node"],
                                best_station,
                                heuristic=heuristic,
                                weight="weight"
                            )

                            build_visual_path(robot)

                # move
                if robot["visual_path"]:

                    robot["pos"] = robot["visual_path"].pop(0)

                # logical checkpoint tracking
                if len(robot["logical_path"]) > 1:

                    next_node = robot["logical_path"][1]

                    node_x, node_y = nodes[next_node]

                    px, py = robot["pos"]

                    if (
                        abs(px - node_x) < 0.01
                        and
                        abs(py - node_y) < 0.01
                    ):

                        robot["logical_path"].pop(0)

                        robot["node"] = robot["logical_path"][0]

                        if len(robot["logical_path"]) > 1:
                            build_visual_path(robot)

                # final parking arrival
                if len(robot["logical_path"]) == 1:

                    final_node = robot["logical_path"][0]

                    if final_node in PARKING_STATIONS:

                        robot["node"] = final_node

                        robot["logical_path"] = []
                        robot["visual_path"] = []

            continue

        if "current_task" in robot:
            task = robot["current_task"]

            # GO TO PICKUP
            if not robot["has_payload"]:

                if robot["node"] != task["PL"]:

                    if len(robot["logical_path"]) <= 1:

                        robot["logical_path"] = nx.astar_path(
                            G,
                            robot["node"],
                            task["PL"],
                            heuristic=heuristic,
                            weight="weight"
                        )

                        build_visual_path(robot)

                    # if not robot["path"]:
                    #     path = nx.astar_path(G, robot["node"], task["PL"], heuristic=heuristic, weight="weight")
                    #     robot["path"] = smooth_path(path, robot["speed"], False)
                    if robot["visual_path"]:

                        robot["pos"] = robot["visual_path"].pop(0)

                    # ---------------------------------
                    # LOGICAL CHECKPOINT TRACK done
                    # ---------------------------------

                    if len(robot["logical_path"]) > 1:

                        next_node = robot["logical_path"][1]

                        node_x, node_y = nodes[next_node]

                        px, py = robot["pos"]

                        if (
                            abs(px - node_x) < 0.01
                            and
                            abs(py - node_y) < 0.01
                        ):

                            # remove previous node
                            robot["logical_path"].pop(0)

                            # update current node
                            robot["node"] = robot["logical_path"][0]

                            update_current_task_remaining_time(name)

                            # print_all_tasks()

                            # rebuild only if more movement remains
                            if len(robot["logical_path"]) > 1:
                                build_visual_path(robot)

                else:
                    robot["wait"] = int(task["pickup_time"] * SIMULATION_FPS)
                    robot["has_payload"] = True

            # GO TO DROPOFF
            else:

                if robot["node"] != task["DL"]:

                    if len(robot["logical_path"]) <= 1:

                        robot["logical_path"] = nx.astar_path(
                            G,
                            task["PL"],
                            task["DL"],
                            heuristic=heuristic,
                            weight="weight"
                        )

                        build_visual_path(robot)

                    # if not robot["path"]:
                    #     path = nx.astar_path(G, task["PL"], task["DL"], heuristic=heuristic, weight="weight")
                    #     robot["path"] = smooth_path(path, robot["speed"], True)
                    if robot["visual_path"]:

                        robot["pos"] = robot["visual_path"].pop(0)

                    # ---------------------------------
                    # LOGICAL CHECKPOINT TRACK done
                    # ---------------------------------

                    if len(robot["logical_path"]) > 1:

                        next_node = robot["logical_path"][1]

                        node_x, node_y = nodes[next_node]

                        px, py = robot["pos"]

                        if (
                            abs(px - node_x) < 0.01
                            and
                            abs(py - node_y) < 0.01
                        ):

                            # remove previous node
                            robot["logical_path"].pop(0)

                            # update current node
                            robot["node"] = robot["logical_path"][0]

                            update_current_task_remaining_time(name)

                            # print_all_tasks()

                            # rebuild only if more movement remains
                            if len(robot["logical_path"]) > 1:
                                build_visual_path(robot)

                else:
                    robot["wait"] = int(task["dropoff_time"] * SIMULATION_FPS)
                    update_task_status(
                        task["id"],
                        "COMPLETED"
                    )

                    update_remaining_time(
                        task["id"],
                        0
                    )

                    robot["queue"].pop(0)

                    robot["real_time_until_free"] = (
                        calculate_robot_rtuf(name)
                    )

                    completion_time = frame / SIMULATION_FPS

                    actual_completion_time = (
                        completion_time
                        - task["actual_start_time"]
                    )

                    update_actual_completion_time(
                        task["id"],
                        actual_completion_time
                    )

                    flow_time = (
                        completion_time
                        - task["created_sim_time"]
                    )

                    lateness = max(
                        0,
                        flow_time - task["deadline"]
                    )

                    evaluation_metrics["cumulative_penalty"] += lateness

                    evaluation_counters["completed_tasks"] += 1

                    if lateness == 0:
                        evaluation_counters["successful_tasks"] += 1

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
        if robot["visual_path"]:

            next_x, next_y = robot["visual_path"][0]

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

            if robot["visual_path"]:

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

        db_rtuf = calculate_robot_rtuf(robot_name)
        db_queue_length = len(get_robot_tasks(robot_name))

        queue_display.append(
            f"{robot_name} Queue | TUF: {db_rtuf:.2f}s | "
            f"Active: {db_queue_length}"
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

    global simulation_done

    if simulation_finished() and not simulation_done:

        simulation_done = True
        finalize_metrics(frame)

        export_metrics(evaluation_metrics)

        print()

        print("==============================")
        print("SIMULATION COMPLETE")
        print("==============================")
        print(
            f"Makespan           : {evaluation_metrics['makespan']:.2f}"
        )
        print(
            f"Computation Time   : {evaluation_metrics['computation_time']:.6f}"
        )
        print(
            f"Cumulative Penalty : {evaluation_metrics['cumulative_penalty']:.2f}"
        )
        print(
            f"Task Success Rate  : {evaluation_metrics['task_success_rate']:.2f}%"
        )
        print("==============================")

        plt.close()



    return r1_visual, r2_visual, r1_status_text, r2_status_text, queue_text

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=1)
plt.gca().invert_yaxis()
plt.show()
