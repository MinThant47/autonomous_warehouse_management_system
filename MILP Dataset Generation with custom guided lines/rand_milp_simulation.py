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
import time
import json

from task_database import (
    initialize_database,
    insert_task,
    update_task_status,
    update_remaining_time,
    update_actual_completion_time,
    calculate_robot_rtuf,
    get_robot_tasks
)
import pandas as pd

# -----------------------------
# EVALUATION METRICS
# -----------------------------

total_solver_time = 0.0
solver_calls = 0

completed_tasks = 0
successful_tasks = 0
cumulative_penalty = 0.0

simulation_makespan = 0.0

# -----------------------------
# GLOBAL TIME SCALE 
# -----------------------------
SIMULATION_FPS = 12.5

file_name = "new_15"
random_seed = 20

initialize_database()
with open(f"Tasks/{file_name}.json", "r") as f:
    scenario_tasks = json.load(f)

scenario_index = 0

MIN_INTERVAL_SECONDS = 1
MAX_INTERVAL_SECONDS = 10

BATCH_INTERVAL_FRAMES = int(
    random.randint(
        MIN_INTERVAL_SECONDS,
        MAX_INTERVAL_SECONDS
    ) * SIMULATION_FPS
)

random.seed(random_seed)

ARRIVAL_PATTERN = [
    random.randint(
        MIN_INTERVAL_SECONDS,
        MAX_INTERVAL_SECONDS
    )
    for _ in range(10000)      # much larger than your task count
]

arrival_index = 0

# 1 graph distance unit = 0.0254 meter (1 inch)
MAP_SCALE = 0.0254
DEFAULT_ROBOT_SPEED = 0.25333
# -----------------------------

initialize_database()

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


def export_metrics(metrics):

    with open(
        f"Metrics/{file_name}_random_seed_{random_seed}_MILP_metrics.json",
        "w"
    ) as f:

        json.dump(
            metrics,
            f,
            indent=4
        )

# -----------------------------
# MILP
# -----------------------------
import gurobipy as gp
from gurobipy import GRB

#between nodes

def compute_travel_time(
    start_node,
    end_node,
    speed,
    G,
    MAP_SCALE
):

    dist = nx.astar_path_length(
        G,
        start_node,
        end_node,
        heuristic=heuristic,
        weight="weight"
    )

    dist_m = dist * MAP_SCALE

    return dist_m / speed

#between start node and task pickup, plus pickup to dropoff  
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

#If a new task arrives now, how long until Robot 1 becomes available?

def compute_current_task_remaining_time(robot):

    if "current_task" not in robot:
        return 0.0

    
    task = robot["current_task"]
    wait_time = robot["wait"] / SIMULATION_FPS

    if robot["has_payload"]:
        # dropoff time = wait time
        if robot["node"] == task["DL"]:
            return wait_time

        # if the robot is currently carrying the payload but not at the dropoff, we need to compute time to dropoff only
        d = nx.astar_path_length(
            G,
            robot["node"],
            task["DL"],
            heuristic=heuristic,
            weight="weight"
        )

        return (
            wait_time
            + (d * MAP_SCALE) / robot["speed"]
            + task["dropoff_time"]
        )

    if robot["node"] == task["PL"]:

        d2 = nx.astar_path_length(
            G,
            task["PL"],
            task["DL"],
            heuristic=heuristic,
            weight="weight"
        )

        return (
            wait_time
            + task["pickup_time"]
            + (d2 * MAP_SCALE) / robot["speed"]
            + task["dropoff_time"]
        )

    d1 = nx.astar_path_length(
        G,
        robot["node"],
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
        wait_time
        + ((d1 + d2) * MAP_SCALE) / robot["speed"]
        + task["pickup_time"]
        + task["dropoff_time"]
    )

# freeze to separate a robot's queue into two parts:
def get_frozen_and_pending_tasks(robot):

    if "current_task" in robot and robot["queue"]:
        return [robot["queue"][0]], robot["queue"][1:]

    return [], list(robot["queue"])


#It returns two values:
# (base_time, base_node)
# base_time = how long until the robot finishes its frozen task
# base_node = where the robot will be after finishing the frozen task
def get_online_sequence_base(robot):

    if "current_task" in robot and robot["queue"]:
        return compute_current_task_remaining_time(robot), robot["queue"][0]["DL"]

    return 0.0, robot["node"]


def evaluate_fixed_robot_queue(robot_name, sim_time):

    robot = robots[robot_name]
    base_time, start_node = get_online_sequence_base(robot)
    frozen_tasks, pending_tasks = get_frozen_and_pending_tasks(robot)

    completion_from_now = 0.0
    cumulative_lateness = 0.0

    if frozen_tasks:
        current_task = frozen_tasks[0]
        completion_from_now = base_time
        current_created = current_task.get("created_sim_time", sim_time)
        cumulative_lateness += max(
            0.0,
            sim_time
            + completion_from_now
            - current_created
            - current_task["deadline"]
        )

    next_start_node = start_node

    for task in pending_tasks:
        task_time = compute_task_time_seconds_from_node(
            next_start_node,
            robot,
            task
        )

        completion_from_now += task_time
        created_time = task.get("created_sim_time", sim_time)
        cumulative_lateness += max(
            0.0,
            sim_time
            + completion_from_now
            - created_time
            - task["deadline"]
        )
        next_start_node = task["DL"]

    return completion_from_now, cumulative_lateness


def solve_pending_sequence_for_robot(
    robot_name,
    pending_tasks,
    sim_time,
    other_robot_completion,
    other_robot_lateness
):
    
    global total_solver_time
    global solver_calls

    robot = robots[robot_name]

    if not pending_tasks:
        return [], 0.0, other_robot_lateness, other_robot_completion

    base_time, base_node = get_online_sequence_base(robot)
    frozen_tasks, _ = get_frozen_and_pending_tasks(robot)
    selected_robot_frozen_lateness = 0.0

    if frozen_tasks:
        frozen_task = frozen_tasks[0]
        selected_robot_frozen_lateness = max(
            0.0,
            sim_time
            + base_time
            - frozen_task.get("created_sim_time", sim_time)
            - frozen_task["deadline"]
        )

    model = gp.Model()
    model.setParam("OutputFlag", 0)
    model.setParam("TimeLimit", 3)

    T = range(len(pending_tasks))
    M = 10000

    travel_init = {}
    travel_between = {}
    task_duration = {}

    for t in T:
        travel_init[t] = compute_travel_time(
            base_node,
            pending_tasks[t]["PL"],
            robot["speed"],
            G,
            MAP_SCALE
        )

        task_duration[t] = (
            compute_travel_time(
                pending_tasks[t]["PL"],
                pending_tasks[t]["DL"],
                robot["speed"],
                G,
                MAP_SCALE
            )
            + pending_tasks[t]["pickup_time"]
            + pending_tasks[t]["dropoff_time"]
        )

    for t1 in T:
        for t2 in T:
            if t1 == t2:
                continue

            travel_between[t1, t2] = compute_travel_time(
                pending_tasks[t1]["DL"],
                pending_tasks[t2]["PL"],
                robot["speed"],
                G,
                MAP_SCALE
            )

    Y = model.addVars(
        T,
        T,
        vtype=GRB.BINARY,
        name="Sequence"
    )

    Start = model.addVars(
        T,
        vtype=GRB.CONTINUOUS,
        name="StartTime"
    )

    End = model.addVars(
        T,
        vtype=GRB.CONTINUOUS,
        name="EndTime"
    )

    Lateness = model.addVars(
        T,
        vtype=GRB.CONTINUOUS,
        name="Lateness"
    )

    Cmax = model.addVar(
        vtype=GRB.CONTINUOUS,
        name="Cmax"
    )

    for t in T:
        model.addConstr(
            Start[t]
            >=
            base_time
            + travel_init[t]
        )

        model.addConstr(
            End[t]
            >=
            Start[t]
            + task_duration[t]
        )

        model.addConstr(
            Cmax >= End[t]
        )

        model.addConstr(
            Lateness[t]
            >=
            sim_time
            + End[t]
            - pending_tasks[t].get("created_sim_time", sim_time)
            - pending_tasks[t]["deadline"]
        )

        model.addConstr(
            Lateness[t] >= 0
        )

    model.addConstr(
        Cmax >= other_robot_completion
    )

    for t1 in T:
        for t2 in T:
            if t1 < t2:
                model.addConstr(
                    Y[t1, t2]
                    + Y[t2, t1]
                    == 1
                )

    for t1 in T:
        for t2 in T:
            if t1 == t2:
                continue

            model.addConstr(
                Start[t2]
                >=
                End[t1]
                + travel_between[t1, t2]
                - M * (1 - Y[t1, t2])
            )

    model.setObjective(
        Cmax
        + 10 * (
            other_robot_lateness
            + selected_robot_frozen_lateness
            + sum(Lateness[t] for t in T)
        ),
        GRB.MINIMIZE
    )

    start_solver = time.perf_counter()
    model.optimize()

    solver_time = time.perf_counter() - start_solver

    total_solver_time += solver_time
    solver_calls += 1

    if model.SolCount == 0:
        raise RuntimeError(
            f"Online MILP could not find a usable solution for {robot_name}. "
            f"Status: {model.Status}"
        )

    ordered_items = [
        {
            "task": pending_tasks[t],
            "start_time": Start[t].x,
            "end_time": End[t].x,
            "lateness": Lateness[t].x,
        }
        for t in T
    ]

    ordered_items.sort(
        key=lambda item: item["start_time"]
    )

    ordered_tasks = [
        item["task"]
        for item in ordered_items
    ]

    return (
        ordered_tasks,
        model.ObjVal,
        (
            sum(Lateness[t].x for t in T)
            + other_robot_lateness
            + selected_robot_frozen_lateness
        ),
        Cmax.x
    )


def solve_online_milp_arrival(task, sim_time):

    task["created_sim_time"] = sim_time

    before_state = snapshot_robot_state()
    candidates = {}

    for robot_name in robots.keys():
        other_robot_name = (
            "R2"
            if robot_name == "R1"
            else "R1"
        )

        frozen_tasks, pending_tasks = get_frozen_and_pending_tasks(
            robots[robot_name]
        )

        candidate_pending_tasks = pending_tasks + [task]

        other_completion, other_lateness = evaluate_fixed_robot_queue(
            other_robot_name,
            sim_time
        )

        ordered_pending, objective_value, lateness, cmax = (
            solve_pending_sequence_for_robot(
                robot_name,
                candidate_pending_tasks,
                sim_time,
                other_completion,
                other_lateness
            )
        )

        candidates[robot_name] = {
            "frozen_tasks": frozen_tasks,
            "ordered_pending": ordered_pending,
            "objective_value": objective_value,
            "lateness": lateness,
            "cmax": cmax,
        }

    selected_robot = min(
        candidates,
        key=lambda robot_name: candidates[robot_name]["objective_value"]
    )

    return selected_robot, candidates[selected_robot], before_state


def apply_online_milp_decision(task, robot_name, decision):

    robot = robots[robot_name]

    interrupt_parking_movement(robot_name)

    task_start_node = (
        robot["queue"][-1]["DL"]
        if robot["queue"]
        else robot["node"]
    )

    task_duration = compute_task_time_seconds_from_node(
        task_start_node,
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

    task["assigned_robot"] = robot_name
    task["group_id"] = f"A{batch_counter:05d}_{robot_name}"

    robot["queue"] = (
        decision["frozen_tasks"]
        + decision["ordered_pending"]
    )

    for rank, queued_task in enumerate(robot["queue"], start=1):
        queued_task["sequence_rank"] = rank

    update_current_task_remaining_time(robot_name)
    robot["time_until_free"] = math.ceil(
        robot["real_time_until_free"] * SIMULATION_FPS
    )

    if robot["queue"]:
        robot["next_node"] = robot["queue"][-1]["DL"]
    else:
        robot["next_node"] = robot["node"]

def snapshot_robot_state():

    return {
        robot_name: {
            "next_node": robot["queue"][-1]["DL"] if robot["queue"] else robot["node"],
            "real_time_until_free": evaluate_fixed_robot_queue(
                robot_name,
                0.0
            )[0],
            "queue_length": len(robot["queue"])
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
        "task_pl_node": task["PL"],
        "task_dl_node": task["DL"],
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
        "pickup_time": task["pickup_time"],
        "dropoff_time": task["dropoff_time"],
        "deadline": task["deadline"],
        "assigned_robot": encode_robot_label(assigned_robot),
        "sequence_rank": sequence_rank
    }


def add_ranker_context_features(df):

    if df.empty:
        return df

    df = df.copy()

    def graph_distance_m(start_node, end_node):
        return (
            nx.astar_path_length(
                G,
                start_node,
                end_node,
                heuristic=heuristic,
                weight="weight"
            )
            * MAP_SCALE
        )

    def graph_travel_time_s(start_node, end_node, speed=DEFAULT_ROBOT_SPEED):
        return graph_distance_m(start_node, end_node) / speed

    df["group_size"] = (
        df.groupby("group_id")["task_id"]
        .transform("count")
    )

    avg_pickup_distances = []
    nearest_pickup_distances = []
    avg_dropoff_to_pickup_distances = []
    nearest_dropoff_to_pickup_distances = []

    for group_id, group in df.groupby("group_id", sort=False):
        group_records = group.to_dict("records")
        n = len(group)

        for i in range(n):
            pickup_dists = []
            dropoff_to_pickup_dists = []

            for j in range(n):
                if i == j:
                    continue

                pickup_dists.append(
                    graph_distance_m(
                        group_records[i]["task_pl_node"],
                        group_records[j]["task_pl_node"]
                    )
                )

                dropoff_to_pickup_dists.append(
                    graph_distance_m(
                        group_records[i]["task_dl_node"],
                        group_records[j]["task_pl_node"]
                    )
                )

            if pickup_dists:
                avg_pickup_distances.append(
                    float(np.mean(pickup_dists))
                )
                nearest_pickup_distances.append(
                    float(np.min(pickup_dists))
                )
                avg_dropoff_to_pickup_distances.append(
                    float(np.mean(dropoff_to_pickup_dists))
                )
                nearest_dropoff_to_pickup_distances.append(
                    float(np.min(dropoff_to_pickup_dists))
                )
            else:
                avg_pickup_distances.append(0.0)
                nearest_pickup_distances.append(0.0)
                avg_dropoff_to_pickup_distances.append(0.0)
                nearest_dropoff_to_pickup_distances.append(0.0)

    df["avg_pickup_distance_to_others"] = avg_pickup_distances
    df["nearest_pickup_distance"] = nearest_pickup_distances
    df["avg_dropoff_to_pickup_distance"] = avg_dropoff_to_pickup_distances
    df["nearest_dropoff_to_pickup_distance"] = nearest_dropoff_to_pickup_distances

    assigned_is_r1 = df["assigned_robot"] == 0

    df["assigned_robot_tuf"] = np.where(
        assigned_is_r1,
        df["r1_tuf"],
        df["r2_tuf"]
    )

    df["other_robot_tuf"] = np.where(
        assigned_is_r1,
        df["r2_tuf"],
        df["r1_tuf"]
    )

    df["assigned_robot_queue_length"] = np.where(
        assigned_is_r1,
        df["r1_queue_length"],
        df["r2_queue_length"]
    )

    df["other_robot_queue_length"] = np.where(
        assigned_is_r1,
        df["r2_queue_length"],
        df["r1_queue_length"]
    )

    df["assigned_robot_dist_to_pickup"] = np.where(
        assigned_is_r1,
        df["r1_dist_to_pickup"],
        df["r2_dist_to_pickup"]
    )

    df["other_robot_dist_to_pickup"] = np.where(
        assigned_is_r1,
        df["r2_dist_to_pickup"],
        df["r1_dist_to_pickup"]
    )

    df["assigned_queue_minus_other_queue"] = (
        df["assigned_robot_queue_length"]
        - df["other_robot_queue_length"]
    )

    df["assigned_tuf_minus_other_tuf"] = (
        df["assigned_robot_tuf"]
        - df["other_robot_tuf"]
    )

    df["pickup_distance_advantage"] = (
        df["other_robot_dist_to_pickup"]
        - df["assigned_robot_dist_to_pickup"]
    )

    df["total_direct_task_distance"] = (
        df["assigned_robot_dist_to_pickup"]
        + df["task_dist_to_dropoff"]
    )

    df["total_direct_task_time"] = (
        df["total_direct_task_distance"]
        / DEFAULT_ROBOT_SPEED
        + df["pickup_time"]
        + df["dropoff_time"]
    )

    df["estimated_completion_time"] = (
        df["assigned_robot_tuf"]
        + df["total_direct_task_time"]
    )

    df["deadline_margin"] = (
        df["deadline"]
        - df["estimated_completion_time"]
    )

    # df["deadline_margin_rank"] = (
    #     df.groupby("group_id")["deadline_margin"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    # df["completion_time_rank"] = (
    #     df.groupby("group_id")["estimated_completion_time"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    min_slack = (
        df.groupby("group_id")["deadline_margin"]
        .transform("min")
    )

    df["deadline_margin_difference"] = (
        df["deadline_margin"]
        - min_slack
    )

    # df["lateness_rank"] = (
    #     df.groupby("group_id")["deadline_margin"]
    #     .rank(
    #         method="dense",
    #         ascending=False
    #     )
    # )

    df["estimated_lateness_if_next"] = np.maximum(
        0.0,
        -df["deadline_margin"]
    )

    df["deadline_margin_ratio"] = np.where(
        df["deadline"] > 0,
        df["deadline_margin"] / df["deadline"],
        0.0
    )

    df["urgency_per_meter"] = np.where(
        df["total_direct_task_distance"] > 0,
        df["deadline"] / df["total_direct_task_distance"],
        df["deadline"]
    )

    df["queue_context_load"] = (
        df["assigned_robot_tuf"]
        + df["assigned_robot_queue_length"]
    )


    # df["assigned_pickup_distance_rank"] = (
    #     df.groupby("group_id")["assigned_robot_dist_to_pickup"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    # df["task_dropoff_distance_rank"] = (
    #     df.groupby("group_id")["task_dist_to_dropoff"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    # df["total_direct_distance_rank"] = (
    #     df.groupby("group_id")["total_direct_task_distance"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    # df["nearest_pickup_rank"] = (
    #     df.groupby("group_id")["nearest_pickup_distance"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    # df["dropoff_to_pickup_rank"] = (
    #     df.groupby("group_id")["nearest_dropoff_to_pickup_distance"]
    #     .rank(
    #         method="dense",
    #         ascending=True
    #     )
    # )

    best_successor_distances = []
    average_successor_distances = []
    best_predecessor_distances = []
    average_predecessor_distances = []
    finish_if_next_values = []
    finish_if_last_values = []
    remaining_queue_time_after_task_values = []
    future_transition_cost_values = []
    predecessor_transition_cost_values = []
    best_successor_time_values = []
    average_successor_time_values = []
    best_predecessor_time_values = []
    average_predecessor_time_values = []

    for group_id, group in df.groupby("group_id", sort=False):
        group_records = group.to_dict("records")
        group_size = len(group_records)

        total_direct_work_seconds = sum(
            (
                row["assigned_robot_dist_to_pickup"]
                + row["task_dist_to_dropoff"]
            )
            / DEFAULT_ROBOT_SPEED
            + row["pickup_time"]
            + row["dropoff_time"]
            for row in group_records
        )

        for current in group_records:
            successor_distances = []
            predecessor_distances = []
            successor_times = []
            predecessor_times = []

            for other in group_records:
                if current["task_id"] == other["task_id"]:
                    continue

                successor_distance = graph_distance_m(
                    current["task_dl_node"],
                    other["task_pl_node"]
                )

                predecessor_distance = graph_distance_m(
                    other["task_dl_node"],
                    current["task_pl_node"]
                )

                successor_distances.append(successor_distance)
                predecessor_distances.append(predecessor_distance)
                successor_times.append(successor_distance / DEFAULT_ROBOT_SPEED)
                predecessor_times.append(predecessor_distance / DEFAULT_ROBOT_SPEED)

            current_direct_work_seconds = (
                current["assigned_robot_dist_to_pickup"]
                + current["task_dist_to_dropoff"]
            ) / DEFAULT_ROBOT_SPEED + current["pickup_time"] + current["dropoff_time"]

            current_task_duration_seconds = (
                current["task_dist_to_dropoff"] / DEFAULT_ROBOT_SPEED
                + current["pickup_time"]
                + current["dropoff_time"]
            )

            finish_if_next = (
                current["assigned_robot_tuf"]
                + current_direct_work_seconds
            )

            if successor_distances:
                best_successor = min(successor_distances)
                average_successor = float(np.mean(successor_distances))
                best_predecessor = min(predecessor_distances)
                average_predecessor = float(np.mean(predecessor_distances))
                best_successor_time = min(successor_times)
                average_successor_time = float(np.mean(successor_times))
                best_predecessor_time = min(predecessor_times)
                average_predecessor_time = float(np.mean(predecessor_times))
            else:
                best_successor = 0.0
                average_successor = 0.0
                best_predecessor = 0.0
                average_predecessor = 0.0
                best_successor_time = 0.0
                average_successor_time = 0.0
                best_predecessor_time = 0.0
                average_predecessor_time = 0.0

            remaining_direct_work_seconds = (
                total_direct_work_seconds
                - current_direct_work_seconds
            )
            remaining_transition_work = (
                average_successor_time * max(0, group_size - 1)
            )

            best_successor_distances.append(best_successor)
            average_successor_distances.append(average_successor)
            best_predecessor_distances.append(best_predecessor)
            average_predecessor_distances.append(average_predecessor)
            best_successor_time_values.append(best_successor_time)
            average_successor_time_values.append(average_successor_time)
            best_predecessor_time_values.append(best_predecessor_time)
            average_predecessor_time_values.append(average_predecessor_time)
            finish_if_next_values.append(finish_if_next)
            finish_if_last_values.append(
                current["assigned_robot_tuf"]
                + remaining_direct_work_seconds
                + best_predecessor_time
                + current_task_duration_seconds
            )
            remaining_queue_time_after_task_values.append(
                remaining_direct_work_seconds
                + remaining_transition_work
            )
            future_transition_cost_values.append(average_successor_time)
            predecessor_transition_cost_values.append(average_predecessor_time)

    df["best_successor_distance"] = best_successor_distances
    df["average_successor_distance"] = average_successor_distances
    df["best_predecessor_distance"] = best_predecessor_distances
    df["average_predecessor_distance"] = average_predecessor_distances
    df["best_successor_time"] = best_successor_time_values
    df["average_successor_time"] = average_successor_time_values
    df["best_predecessor_time"] = best_predecessor_time_values
    df["average_predecessor_time"] = average_predecessor_time_values
    df["current_base_to_pickup"] = df["assigned_robot_dist_to_pickup"]
    df["current_base_to_pickup_time"] = (
        df["assigned_robot_dist_to_pickup"]
        / DEFAULT_ROBOT_SPEED
    )
    df["finish_if_next"] = finish_if_next_values
    df["finish_if_last"] = finish_if_last_values
    df["remaining_queue_time_after_task"] = remaining_queue_time_after_task_values
    df["future_transition_cost"] = future_transition_cost_values
    df["predecessor_transition_cost"] = predecessor_transition_cost_values
    df["finish_position_span"] = (
        df["finish_if_last"]
        - df["finish_if_next"]
    )

    df["rank_label"] = (
        df.groupby("group_id")["sequence_rank"]
        .transform("max")
        - df["sequence_rank"]
    )

    return df


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
        robot["real_time_until_free"] = 0.0
        robot["next_node"] = robot["node"]
        return

    next_start_node = robot["node"]

    for queue_index, task in enumerate(robot["queue"]):

        is_current_task = (
            queue_index == 0
            and "current_task" in robot
            and task["id"] == robot["current_task"]["id"]
        )

        if is_current_task:

            remaining_time = compute_current_task_remaining_time(robot)

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

    if robot["queue"]:
        robot["next_node"] = robot["queue"][-1]["DL"]
    else:
        robot["next_node"] = robot["node"]
# -----------------------------
# UPDATE LOOP
# -----------------------------
def update(frame):
    global task_counter
    global batch_counter
    global current_task_visual
    global scenario_index

    global total_solver_time
    global solver_calls
    global completed_tasks
    global successful_tasks
    global cumulative_penalty
    global simulation_makespan

    global BATCH_INTERVAL_FRAMES
    global arrival_index


    task_counter += 1

    # DISPATCH WINDOW
    if task_counter >= BATCH_INTERVAL_FRAMES:
        task_counter = 0

        arrival_seconds = ARRIVAL_PATTERN[arrival_index]
        arrival_index += 1

        print(f"Arrival Interval = {arrival_seconds} s")

        BATCH_INTERVAL_FRAMES = int(
            arrival_seconds * SIMULATION_FPS
        )


        if scenario_index >= len(scenario_tasks):
            print()
            if (
                scenario_index >= len(scenario_tasks)
                and
                all(len(r["queue"]) == 0 for r in robots.values())
                and
                all("current_task" not in r for r in robots.values())
            ):

                print("\n==============================")
                print("ONLINE MILP COMPLETE")
                print("==============================")

                print(
                    f"Makespan           : {simulation_makespan:.2f}"
                )

                print(
                    f"Total Solver Time  : {total_solver_time:.6f}"
                )

                print(
                    f"Average Solve Time : "
                    f"{total_solver_time / solver_calls:.6f}"
                )

                print(
                    f"Cumulative Penalty : "
                    f"{cumulative_penalty:.2f}"
                )

                print(
                    f"Task Success Rate  : "
                    f"{100*successful_tasks/completed_tasks:.2f}%"
                )

                print("==============================")

                metrics = {
                    "makespan": round(simulation_makespan, 2),
                    "computation_time": round(total_solver_time, 6),
                    "cumulative_penalty": round(cumulative_penalty, 2),
                    "task_success_rate": round(
                        100 * successful_tasks / completed_tasks, 2
                    )
                }
                export_metrics(metrics)

                plt.close()
            
            
            print("DISPATCH WINDOW: scenario exhausted")

        else:
            batch_counter += 1

            task = scenario_tasks[scenario_index]
            scenario_index += 1
            current_task_visual = task

            sim_time = frame / SIMULATION_FPS

            

            selected_robot, decision, robot_state = solve_online_milp_arrival(
                task,
                sim_time
            )

            apply_online_milp_decision(
                task,
                selected_robot,
                decision
            )

            pending_group = decision["ordered_pending"]

            assigned_robot_state = snapshot_robot_state()

            print(
                "\n=============================="
            )
            print(
                f"ONLINE MILP ARRIVAL A{batch_counter:05d}"
            )
            print(
                f"T{task['id']} | {task['PL']} -> {task['DL']} "
                f"| deadline {task['deadline']}s"
            )
            print(
                f"Assigned robot: {selected_robot}"
            )
            print(
                f"Objective: {decision['objective_value']:.2f} | "
                f"Cmax: {decision['cmax']:.2f} | "
                f"Lateness: {decision['lateness']:.2f}"
            )
            print(
                "Optimized pending queue: "
                + " | ".join(
                    f"{rank}. T{pending_task['id']}"
                    for rank, pending_task in enumerate(
                        pending_group,
                        start=1
                    )
                )
            )
            print(
                "\nWorkload after assignment:"
            )
            for robot_name in robots.keys():
                print(
                    f"  {robot_name}: "
                    f"RTUF={assigned_robot_state[robot_name]['real_time_until_free']}s, "
                    f"queue={assigned_robot_state[robot_name]['queue_length']}"
                )
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
                time.perf_counter()
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

                    actual_completion_time = (
                        time.perf_counter()
                        - task["actual_start_time"]
                    )

                    update_actual_completion_time(
                        task["id"],
                        actual_completion_time
                    )

                    # -----------------------------
                    # EVALUATION
                    # -----------------------------
                    completed_tasks += 1

                    completion_time = frame / SIMULATION_FPS

                    simulation_makespan = max(
                        simulation_makespan,
                        completion_time
                    )

                    lateness = max(
                        0,
                        completion_time
                        - task["created_sim_time"]
                        - task["deadline"]
                    )

                    cumulative_penalty += lateness

                    if lateness == 0:
                        successful_tasks += 1

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

    return r1_visual, r2_visual, r1_status_text, r2_status_text, queue_text

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=0.1)
plt.gca().invert_yaxis()
plt.show()
