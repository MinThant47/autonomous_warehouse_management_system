import networkx as nx
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import json
import joblib
import pandas as pd
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.patches import FancyBboxPatch
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
)

queue_robot_icons = {
    "R1": plt.imread("robot1.png"),
    "R2": plt.imread("robot2.png"),
}

QUEUE_ICON_ZOOM = 0.018      # QUEUE ICON Size
QUEUE_ICON_X = 0.05         # Horizontal position
QUEUE_ICON_Y = 1.08         # Vertical position
# -----------------------------
# GLOBAL TIME SCALE 
# -----------------------------
SIMULATION_FPS = 12.5
simulation_done = False
simulation_complete_frame = None
FINAL_HOLD_SECONDS = 8
FINAL_HOLD_FRAMES = int(FINAL_HOLD_SECONDS * SIMULATION_FPS)

# 1 graph distance unit = 0.0254 meter (1 inch)
MAP_SCALE = 0.0254
DEFAULT_ROBOT_SPEED = 0.25333/2
# -----------------------------

file_name = "old_10"
random_seed = 18

initialize_database()
with open(f"Tasks/{file_name}.json", "r") as f:
    scenario_tasks = json.load(f)

scenario_index = 0

# 4-6 good
MIN_INTERVAL_SECONDS = 1
MAX_INTERVAL_SECONDS = 8

random.seed(random_seed)

ARRIVAL_PATTERN = [
    random.randint(
        MIN_INTERVAL_SECONDS,
        MAX_INTERVAL_SECONDS
    )
    for _ in range(100)      # much larger than your task count
]

arrival_index = 0

BATCH_INTERVAL_FRAMES = int(
    ARRIVAL_PATTERN[arrival_index] * SIMULATION_FPS
)

classifier = joblib.load(
    "Classifiers/xgb_classifier_deadline.pkl"
)

ranker = joblib.load(
    "Rankers/tr_kkkw_best_xgb_ranker.pkl"
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
            "speed": DEFAULT_ROBOT_SPEED
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
            "speed": DEFAULT_ROBOT_SPEED
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

# Adds missing feature columns with zero values and returns only the required model features in the correct order.

def select_model_features(df, feature_columns):

    for column in feature_columns:
        if column not in df.columns:
            df[column] = 0

    return df[feature_columns]


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
    return select_model_features(df, CLASSIFIER_COLUMNS)

def build_ranker_feature_rows(robot_name):

    robot = robots[robot_name]

    fixed_prefix = []
    pending_tasks = robot["queue"]

    if "current_task" in robot and robot["queue"]:
        fixed_prefix = [robot["queue"][0]]
        pending_tasks = robot["queue"][1:]

    if len(pending_tasks) <= 1:
        return pd.DataFrame()

    robot_state = snapshot_robot_state()

    simulated_r1_tuf = robot_state["R1"]["real_time_until_free"]
    simulated_r2_tuf = robot_state["R2"]["real_time_until_free"]

    feature_rows = []

    for batch_position, task in enumerate(pending_tasks, start=1):

        plx, ply = nodes[task["PL"]]
        dlx, dly = nodes[task["DL"]]
        r1_next = robot_state["R1"]["next_node"]
        r2_next = robot_state["R2"]["next_node"]

        r1_dist = graph_distance_m(
            r1_next,
            task["PL"]
        )

        r2_dist = graph_distance_m(
            r2_next,
            task["PL"]
        )

        task_dist = graph_distance_m(
            task["PL"],
            task["DL"]
        )

        r1x, r1y = nodes[r1_next]
        r2x, r2y = nodes[r2_next]

        feature_rows.append({

            "task": task,
            "task_pl_node": task["PL"],
            "task_dl_node": task["DL"],

            "batch_position": batch_position,
            "batch_size": len(pending_tasks),

            "r1_next_x": r1x * MAP_SCALE,
            "r1_next_y": r1y * MAP_SCALE,

            "r2_next_x": r2x * MAP_SCALE,
            "r2_next_y": r2y * MAP_SCALE,

            "r1_tuf": simulated_r1_tuf,
            "r2_tuf": simulated_r2_tuf,

            "r1_queue_length": robot_state["R1"]["queue_length"],
            "r2_queue_length": robot_state["R2"]["queue_length"],

            "tuf_difference":
                simulated_r1_tuf
                - simulated_r2_tuf,

            "task_pl_x": plx * MAP_SCALE,
            "task_pl_y": ply * MAP_SCALE,

            "task_dl_x": dlx * MAP_SCALE,
            "task_dl_y": dly * MAP_SCALE,

            "r1_dist_to_pickup": r1_dist,
            "r2_dist_to_pickup": r2_dist,

            "task_dist_to_dropoff": task_dist,

            "task_type": task["type"],
            "pickup_time": task["pickup_time"],
            "dropoff_time": task["dropoff_time"],
            "deadline": task["deadline"],
            "assigned_robot": encode_robot_label(robot_name),
        })

    df = pd.DataFrame(feature_rows)

    df["group_size"] = len(df)

    avg_pickup = []
    nearest_pickup = []
    avg_dropoff_pickup = []
    nearest_dropoff_pickup = []

    # to avoid repeated DataFrame lookups, convert to list of dicts
    records = df.to_dict("records")

    for i in range(len(records)):
        pickup_distances = []
        dropoff_pickup_distances = []

        for j in range(len(records)):

            # Distance(A,A)=0 is meaningless. So skip it.
            if i == j:
                continue

            pickup_distances.append(
                graph_distance_m(
                    records[i]["task_pl_node"],
                    records[j]["task_pl_node"]
                )
            )

            dropoff_pickup_distances.append(
                graph_distance_m(
                    records[i]["task_dl_node"],
                    records[j]["task_pl_node"]
                )
            )

        if pickup_distances:

            avg_pickup.append(np.mean(pickup_distances))
            nearest_pickup.append(np.min(pickup_distances))

            avg_dropoff_pickup.append(
                np.mean(dropoff_pickup_distances)
            )

            nearest_dropoff_pickup.append(
                np.min(dropoff_pickup_distances)
            )

        else:

            avg_pickup.append(0)
            nearest_pickup.append(0)

            avg_dropoff_pickup.append(0)
            nearest_dropoff_pickup.append(0)

    df["avg_pickup_distance_to_others"] = avg_pickup
    df["nearest_pickup_distance"] = nearest_pickup

    df["avg_dropoff_to_pickup_distance"] = avg_dropoff_pickup
    df["nearest_dropoff_to_pickup_distance"] = nearest_dropoff_pickup

    assigned_is_r1 = robot_name == "R1"

    df["assigned_robot_tuf"] = (
        df["r1_tuf"]
        if assigned_is_r1
        else df["r2_tuf"]
    )

    df["other_robot_tuf"] = (
        df["r2_tuf"]
        if assigned_is_r1
        else df["r1_tuf"]
    )

    df["assigned_robot_queue_length"] = (
        df["r1_queue_length"]
        if assigned_is_r1
        else df["r2_queue_length"]
    )

    df["other_robot_queue_length"] = (
        df["r2_queue_length"]
        if assigned_is_r1
        else df["r1_queue_length"]
    )

    df["assigned_robot_dist_to_pickup"] = (
        df["r1_dist_to_pickup"]
        if assigned_is_r1
        else df["r2_dist_to_pickup"]
    )

    df["other_robot_dist_to_pickup"] = (
        df["r2_dist_to_pickup"]
        if assigned_is_r1
        else df["r1_dist_to_pickup"]
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

    df["deadline_margin_difference"] = (
        df["deadline_margin"]
        - df["deadline_margin"].min()
    )

    df["lateness_rank"] = (
        df["deadline_margin"]
        .rank(method="dense", ascending=False)
    )

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

    # Nonsense Features

    df["queue_context_load"] = (
        df["assigned_robot_tuf"]
        + df["assigned_robot_queue_length"]
    )

    best_successor_distances = []
    average_successor_distances = []
    best_predecessor_distances = []
    average_predecessor_distances = []
    best_successor_time_values = []
    average_successor_time_values = []
    best_predecessor_time_values = []
    average_predecessor_time_values = []
    finish_if_next_values = []
    finish_if_last_values = []
    remaining_queue_time_after_task_values = []
    future_transition_cost_values = []
    predecessor_transition_cost_values = []

    records = df.to_dict("records")
    total_direct_work_seconds = sum(
        row["total_direct_task_time"]
        for row in records
    )

    for current in records:
        successor_distances = [] # remove this later
        predecessor_distances = []
        successor_times = []
        predecessor_times = []

        for other in records:
            if current["task"]["id"] == other["task"]["id"]:
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
            - current["total_direct_task_time"]
        )

        remaining_transition_work = (
            average_successor_time
            * max(0, len(records) - 1)
        )

        best_successor_distances.append(best_successor)
        average_successor_distances.append(average_successor)
        best_predecessor_distances.append(best_predecessor)
        average_predecessor_distances.append(average_predecessor)
        best_successor_time_values.append(best_successor_time)
        average_successor_time_values.append(average_successor_time)
        best_predecessor_time_values.append(best_predecessor_time)
        average_predecessor_time_values.append(average_predecessor_time)
        finish_if_next_values.append(
            current["assigned_robot_tuf"]
            + current["total_direct_task_time"]
        )
        finish_if_last_values.append(
            current["assigned_robot_tuf"]
            + remaining_direct_work_seconds
            + best_predecessor_time
            + (
                current["task_dist_to_dropoff"]
                / DEFAULT_ROBOT_SPEED
                + current["pickup_time"]
                + current["dropoff_time"]
            )
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

    return df
    
def rerank_robot_queue(robot_name):

    robot = robots[robot_name]

    fixed_prefix = []
    pending_tasks = robot["queue"]

    if "current_task" in robot and robot["queue"]:
        fixed_prefix = [robot["queue"][0]]
        pending_tasks = robot["queue"][1:]

    before_pending_positions = {
        task["id"]: position
        for position, task in enumerate(pending_tasks, start=1)
    }

    if len(pending_tasks) <= 1:
        for rank, task in enumerate(robot["queue"], start=1):
            task["sequence_rank"] = rank

        for position, task in enumerate(pending_tasks, start=1):
            task["pending_rank"] = position
            task["previous_pending_rank"] = before_pending_positions.get(
                task["id"],
                position
            )
            task["rank_move"] = "same"
            task["rank_delta"] = 0

        update_current_task_remaining_time(robot_name)
        robot["next_node"] = (
            robot["queue"][-1]["DL"]
            if robot["queue"]
            else robot["node"]
        )
        return

    print()
    print("=" * 60)
    print(robot_name)

    print("Before")

    for t in pending_tasks:
        print(
            f"Task {t['id']:2d}  Deadline={t['deadline']:3d}"
        )

    df = build_ranker_feature_rows(robot_name)

    if df.empty:
        return

    ranker_features = select_model_features(
        df.copy(),
        RANKER_COLUMNS
    )

    prediction_start = time.perf_counter()

    scores = ranker.predict(
        ranker_features
    )



    evaluation_metrics["computation_time"] += (
        time.perf_counter() - prediction_start
    )

    df["score"] = scores

    for _, row in df.iterrows():
        task = row["task"]
        task["ranker_score"] = float(row["score"])
        task["previous_pending_rank"] = before_pending_positions.get(
            task["id"]
        )

    # ----------------------------------
    # CONFIDENCE THRESHOLD
    # ----------------------------------
    score_range = (
        df["score"].max()
        - df["score"].min()
    )

    print(
        f"Score Range = {score_range:.4f}"
    )

    if len(pending_tasks) == 2 and score_range < 1.5:
        print(
            f"Skip rerank "
            f"Pending Tasks={len(pending_tasks)}, range={score_range:.4f})"
        )
        return

    if score_range < 0.8:

        print(
            f"Skip rerank "
            f"(range={score_range:.4f})"
        )

        for position, task in enumerate(pending_tasks, start=1):
            task["pending_rank"] = position
            task["previous_pending_rank"] = before_pending_positions.get(
                task["id"],
                position
            )
            task["rank_move"] = "same"
            task["rank_delta"] = 0

        update_current_task_remaining_time(robot_name)

        robot["next_node"] = (
            robot["queue"][-1]["DL"]
            if robot["queue"]
            else robot["node"]
        )

        return

    print()

    print("Scores")


    for _, row in df.iterrows():

        print(
            f"Task {row['task']['id']:2d}"
            f"  Score={row['score']:.5f}"
            f"  Deadline={row['deadline']}"
            f"  Margin={row['deadline_margin']:.2f}"
        )

    df = df.sort_values(
        "score",
        ascending=False
    )

    robot["queue"] = fixed_prefix + list(df["task"])

    for rank, task in enumerate(robot["queue"], start=1):
        task["sequence_rank"] = rank

    for new_pending_rank, task in enumerate(
        robot["queue"][len(fixed_prefix):],
        start=1
    ):
        old_pending_rank = before_pending_positions.get(
            task["id"],
            new_pending_rank
        )
        rank_delta = old_pending_rank - new_pending_rank

        if rank_delta > 0:
            rank_move = "up"
        elif rank_delta < 0:
            rank_move = "down"
        else:
            rank_move = "same"

        task["pending_rank"] = new_pending_rank
        task["previous_pending_rank"] = old_pending_rank
        task["rank_move"] = rank_move
        task["rank_delta"] = rank_delta

    update_current_task_remaining_time(robot_name)
    robot["next_node"] = robot["queue"][-1]["DL"]

    print()
    print("After")

    for t in robot["queue"][len(fixed_prefix):]:
        print(
            f"Task {t['id']:2d}  Deadline={t['deadline']:3d}"
        )

    print("=" * 60)

def get_ranker_feature_names(model):

    if hasattr(model, "feature_names_in_"):
        return list(model.feature_names_in_)

    booster = model.get_booster()

    if booster.feature_names:
        return list(booster.feature_names)

    raise RuntimeError("Ranker feature names not found.")

RANKER_COLUMNS = get_ranker_feature_names(ranker)
print("Ranker feature columns:")
print(RANKER_COLUMNS)

def apply_xgboost_fifo(
    task,
    classifier
):

    robot_state = snapshot_robot_state()

    print("\n==============================")
    print("XGBOOST CLASSIFIER + RANKER ASSIGNMENT")
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

    evaluation_counters["total_tasks_generated"] += 1

    task["assigned_robot"] = robot_name

    robot["queue"].append(task)

    rerank_robot_queue(robot_name)

    update_current_task_remaining_time(robot_name)

    robot["time_until_free"] = math.ceil(
        robot["real_time_until_free"]
        * SIMULATION_FPS
    )

    robot["next_node"] = (
        robot["queue"][-1]["DL"]
        if robot["queue"]
        else robot["node"]
    )

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


def compute_current_task_remaining_time(robot):

    if "current_task" not in robot:
        return 0.0

    task = robot["current_task"]
    wait_time = robot["wait"] / SIMULATION_FPS

    if robot["has_payload"]:

        if robot["node"] == task["DL"]:
            return wait_time

        return (
            wait_time
            + graph_travel_time_s(
                robot["node"],
                task["DL"],
                robot["speed"]
            )
            + task["dropoff_time"]
        )

    if robot["node"] == task["PL"]:

        return (
            wait_time
            + task["pickup_time"]
            + graph_travel_time_s(
                task["PL"],
                task["DL"],
                robot["speed"]
            )
            + task["dropoff_time"]
        )

    return (
        wait_time
        + compute_task_time_seconds_from_node(
            robot["node"],
            robot,
            task
        )
    )


def evaluate_fixed_robot_queue(robot_name):

    robot = robots[robot_name]

    if not robot["queue"]:
        return 0.0

    total_time = 0.0
    next_start_node = robot["node"]

    for queue_index, task in enumerate(robot["queue"]):
        is_current_task = (
            queue_index == 0
            and "current_task" in robot
            and task["id"] == robot["current_task"]["id"]
        )

        if is_current_task:
            task_time = compute_current_task_remaining_time(robot)
        else:
            task_time = compute_task_time_seconds_from_node(
                next_start_node,
                robot,
                task
            )

        total_time += task_time
        next_start_node = task["DL"]

    return total_time


def snapshot_robot_state():

    return {
        robot_name: {
            "next_node": robot["queue"][-1]["DL"] if robot["queue"] else robot["node"],
            "real_time_until_free": evaluate_fixed_robot_queue(robot_name),
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

def apply_batch_assignments(batch_tasks, assignments, priorities):

    for robot_name in robots.keys():

        ordered_task_indexes = [
            task_index
            for task_index, assigned_robot in assignments.items()
            if assigned_robot == robot_name
        ]

        ordered_task_indexes.sort(
            key=lambda task_index: priorities[task_index]
        )

        if not ordered_task_indexes:
            continue

        interrupt_parking_movement(robot_name)

        robot = robots[robot_name]
        next_node = robot["next_node"]

        for task_index in ordered_task_indexes:

            task = batch_tasks[task_index]
            task_duration = compute_task_time_seconds_from_node(
                next_node,
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
            task["sequence_rank"] = priorities[task_index]
            task["group_id"] = (
                f"B{batch_counter:05d}_{robot_name}"
            )

            robot["queue"].append(task)
            robot["real_time_until_free"] += task_duration
            robot["time_until_free"] = math.ceil(
                robot["real_time_until_free"] * SIMULATION_FPS
            )
            robot["next_node"] = task["DL"]

            next_node = task["DL"]
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

# --------------------------
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

fig = plt.figure(figsize=(16, 10))
dashboard = fig.add_gridspec(
    2,
    2,
    width_ratios=[3.2, 1.8],
    height_ratios=[1, 1],
    wspace=0.08,
    hspace=0.16
)

ax = fig.add_subplot(dashboard[:, 0])
r1_queue_ax = fig.add_subplot(dashboard[0, 1])
r2_queue_ax = fig.add_subplot(dashboard[1, 1])

# Extra left margin so the legend (anchored just outside the map's
# left edge) has room to render without being clipped or overlapping
# the warehouse graph.
fig.subplots_adjust(left=0.16)

queue_axes = {
    "R1": r1_queue_ax,
    "R2": r2_queue_ax
}

for queue_axis in queue_axes.values():
    queue_axis.set_xlim(0, 1)
    queue_axis.set_ylim(1, 0)
    queue_axis.set_xticks([])
    queue_axis.set_yticks([])
    queue_axis.set_facecolor("#f7f8fb")

    for spine in queue_axis.spines.values():
        spine.set_color("#d0d5dd")

# -----------------------------
# ROBOT IMAGES
# -----------------------------

robot_images = {
    name: Image.open(f"robot{1 if name == 'R1' else 2}.png").convert("RGBA")
    for name in ("R1", "R2")
}


def split_current_and_pending(robot):
    if "current_task" in robot and robot["queue"]:
        return robot["queue"][0], robot["queue"][1:]

    return None, list(robot["queue"])


queue_cards = {
    "R1": {},
    "R2": {}
}

# Queue cards are drawn directly on each robot's dedicated
# panel axis (r1_queue_ax / r2_queue_ax), which is set up with
# xlim(0, 1) / ylim(1, 0). So all card geometry below is in
# axes-fraction units, NOT warehouse map units.
CARD_X = 0.05
CARD_W = 0.9
CARD_H = 0.17
CARD_SPACING = 0.195

QUEUE_Y_START = 0.04

# How many card "slots" actually fit in a queue panel before the
# bottom of a card would run past the axis (y=1, since ylim is
# inverted to (1, 0)). One slot is reserved for a "+N more"
# footer whenever the real task count exceeds this.
BOTTOM_MARGIN = 0.02
MAX_VISIBLE_SLOTS = max(
    1,
    int(
        (1.0 - BOTTOM_MARGIN - QUEUE_Y_START - CARD_H)
        / CARD_SPACING
    ) + 1
)


def rank_move_text(task):
    move = task.get("rank_move", "same")
    rank_delta = task.get("rank_delta", 0)

    if move == "up":
        return f"Priority up {abs(rank_delta)}"

    if move == "down":
        return f"Priority down {abs(rank_delta)}"

    return "Priority unchanged"


def queue_card_color(robot_name, task, is_current):
    if is_current:
        return {
            "R1": "#fff4bf",
            "R2": "#d9ecff",
        }[robot_name]

    movement_color = {
        "up": "#c9f7d5",
        "down": "#ffd0d0",
    }.get(
        task.get("rank_move", "same")
    )

    if movement_color:
        return movement_color

    return {
        "R1": "#fffdf2",
        "R2": "#f1f7ff",
    }[robot_name]


def queue_card_text(task, is_current):

    if is_current:
        move_text = "In progress"
    else:
        move_text = rank_move_text(task)

    margin = task.get("deadline_margin")
    margin_text = (
        f"Margin {margin:.0f}s"
        if margin is not None
        else f"{task['PL']} → {task['DL']}"
    )

    return (
        f"Task {task['id']}  ·  {move_text}\n"
        f"{margin_text}"
    )


OVERFLOW_KEY = "__overflow__"


def draw_queue(robot_name):
    robot = robots[robot_name]
    current_task, pending_tasks = split_current_and_pending(robot)
    cards = queue_cards[robot_name]
    panel_ax = queue_axes[robot_name]

    tasks = []

    if current_task:
        tasks.append((current_task, True))

    tasks.extend(
        (task, False)
        for task in pending_tasks
    )

    overflow_count = 0

    if len(tasks) > MAX_VISIBLE_SLOTS:
        # Reserve the last slot for a "+N more" footer card instead
        # of letting real cards run past the bottom of the panel.
        visible_tasks = tasks[: MAX_VISIBLE_SLOTS - 1]
        overflow_count = len(tasks) - len(visible_tasks)
    else:
        visible_tasks = tasks

    used_task_ids = set()

    for index, (task, is_current) in enumerate(visible_tasks):
        task_id = task["id"]
        target_y = (
            QUEUE_Y_START
            + index * CARD_SPACING
        )

        used_task_ids.add(task_id)

        if task_id not in cards:
            box = FancyBboxPatch(
                (
                    CARD_X,
                    target_y
                ),
                CARD_W,
                CARD_H,
                boxstyle="round,pad=0.02",
                facecolor="white",
                edgecolor="black",
                linewidth=1,
                zorder=10
            )

            text = panel_ax.text(
                CARD_X + 0.03,
                target_y + CARD_H / 2,
                "",
                fontsize=10,
                va="center",
                ha="left",
                zorder=11
            )

            panel_ax.add_patch(box)

            cards[task_id] = {
                "box": box,
                "text": text,
                "y": target_y,
                "target": target_y,
                "text_x": CARD_X + 0.03
            }

        cards[task_id]["target"] = target_y
        cards[task_id]["box"].set_facecolor(
            queue_card_color(robot_name, task, is_current)
        )
        cards[task_id]["box"].set_edgecolor(
            (
                "#946200"
                if robot_name == "R1"
                else "#1d5f9f"
            )
            if is_current
            else (
                "#b68a00"
                if robot_name == "R1"
                else "#4b86c5"
            )
        )
        cards[task_id]["text"].set_text(
            queue_card_text(task, is_current)
        )

    if overflow_count > 0:
        index = len(visible_tasks)
        target_y = (
            QUEUE_Y_START
            + index * CARD_SPACING
        )

        used_task_ids.add(OVERFLOW_KEY)

        if OVERFLOW_KEY not in cards:
            box = FancyBboxPatch(
                (
                    CARD_X,
                    target_y
                ),
                CARD_W,
                CARD_H,
                boxstyle="round,pad=0.02",
                facecolor="#e8eaf0",
                edgecolor="#8a8f9c",
                linewidth=1,
                linestyle="dashed",
                zorder=10
            )

            text = panel_ax.text(
                CARD_X + CARD_W / 2,
                target_y + CARD_H / 2,
                "",
                fontsize=10,
                va="center",
                ha="center",
                style="italic",
                color="#555b66",
                zorder=11
            )

            panel_ax.add_patch(box)

            cards[OVERFLOW_KEY] = {
                "box": box,
                "text": text,
                "y": target_y,
                "target": target_y,
                "text_x": CARD_X + CARD_W / 2
            }

        cards[OVERFLOW_KEY]["target"] = target_y
        cards[OVERFLOW_KEY]["text"].set_text(
            f"+{overflow_count} more"
        )

    for task_id in list(cards.keys()):
        if task_id not in used_task_ids:
            cards[task_id]["box"].remove()
            cards[task_id]["text"].remove()
            del cards[task_id]

    for card in cards.values():
        card["y"] += (
            card["target"]
            - card["y"]
        ) * 0.25

        card["box"].set_y(card["y"])
        card["text"].set_position(
            (
                card["text_x"],
                card["y"] + CARD_H / 2
            )
        )


def draw_all_queues():
    draw_queue("R1")
    draw_queue("R2")


def queue_artists():
    artists = []

    for cards in queue_cards.values():
        for card in cards.values():
            artists.extend(
                [
                    card["box"],
                    card["text"]
                ]
            )

    return artists


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
nx.draw_networkx_edges(G, pos, ax=ax, width=2, edge_color="black")
nx.draw_networkx_nodes(G, pos, ax=ax, node_color=node_colors, node_size=50)

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
    ax=ax,
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

ax.set_xlim(
    min(xs) - 5,
    max(xs) + 5
) #plt.xlim(5, 55)
ax.set_ylim(min(ys) - 5, max(ys) + 5) #plt.ylim(10, 45)

ax.set_title("Warehouse Graph")
ax.set_aspect("equal")
ax.grid(False)
for spine in ax.spines.values():
    spine.set_visible(False)

route_lines = {
    "R1": ax.plot([], [], color="#f28c28", linewidth=5, alpha=0.95,
                  solid_capstyle="round", solid_joinstyle="round", zorder=1.5,
                  label="R1 route")[0],
    "R2": ax.plot([], [], color="#2878c8", linewidth=5, alpha=0.95,
                  solid_capstyle="round", solid_joinstyle="round", zorder=1.5,
                  label="R2 route")[0],
}
route_markers = {
    "R1": ax.scatter([], [], s=58, color="#f28c28", edgecolors="white",
                     linewidths=0.8, zorder=3),
    "R2": ax.scatter([], [], s=58, color="#2878c8", edgecolors="white",
                     linewidths=0.8, zorder=3),
}

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

ax.legend(
    handles=legend,
    loc='upper right',
    bbox_to_anchor=(-0.02, 1.0),
    bbox_transform=ax.transAxes,
    borderaxespad=0,
    fontsize=8,
    frameon=True
)

# r1_queue_ax.set_title("R1 Queue", fontsize=10, weight="bold", loc="left")
# r2_queue_ax.set_title("R2 Queue", fontsize=10, weight="bold", loc="left")

for robot_name, queue_ax in zip(
    ["R1", "R2"],
    [r1_queue_ax, r2_queue_ax]
):

    # Robot image
    icon = OffsetImage(
        queue_robot_icons[robot_name],
        zoom=QUEUE_ICON_ZOOM
    )

    ab = AnnotationBbox(
        icon,
        (QUEUE_ICON_X, QUEUE_ICON_Y),
        xycoords="axes fraction",
        frameon=False
    )

    queue_ax.add_artist(ab)

    # Queue title
    queue_ax.text(
        0.13,
        1.08,
        f"{robot_name} Queue",
        transform=queue_ax.transAxes,
        fontsize=12,
        fontweight="bold",
        va="center"
    )

# -----------------------------
# ROBOT VISUALS
# -----------------------------
r1_imgbox = OffsetImage(np.zeros((10,10,4)), zoom=0.03)
r2_imgbox = OffsetImage(np.zeros((10,10,4)), zoom=0.03)

r1_visual = AnnotationBbox(r1_imgbox, (0,0), frameon=False)
r2_visual = AnnotationBbox(r2_imgbox, (0,0), frameon=False)
r1_visual.set_zorder(10)
r2_visual.set_zorder(10)

ax.add_artist(r1_visual)
ax.add_artist(r2_visual)

def export_metrics(metrics):

    with open(
        f"Metrics/1-10/{file_name}_random_seed_{random_seed}_xgboost_ranker_metrics.json",
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

        if robot["wait"] > 0:
            return False

        if robot["visual_path"]:
            return False

        if len(robot["logical_path"]) > 1:
            return False

        if robot.get("time_until_free", 0) > 0:
            return False

        if robot["node"] not in PARKING_STATIONS:
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

        planned_nodes = robot.get("logical_path", [])
        route_points = [(x, y)]
        if len(planned_nodes) > 1:
            route_points.extend(nodes[node] for node in planned_nodes[1:])
        route_x = [point[0] for point in route_points]
        route_y = [point[1] for point in route_points]
        route_lines[robot_name].set_data(route_x, route_y)
        route_markers[robot_name].set_offsets(
            np.asarray([nodes[node] for node in planned_nodes[1:]]).reshape(-1, 2)
        )

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

    draw_all_queues()

    global simulation_done
    global simulation_complete_frame

    if simulation_finished() and not simulation_done:

        simulation_done = True
        simulation_complete_frame = frame
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
        print(
            f"Final view will stay open for {FINAL_HOLD_SECONDS}s."
        )
        print("==============================")

    if (
        simulation_done
        and simulation_complete_frame is not None
        and frame - simulation_complete_frame >= FINAL_HOLD_FRAMES
    ):
        plt.close()



    return (
        r1_visual,
        r2_visual,
        *route_lines.values(),
        *route_markers.values(),
        *queue_artists()
    )

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=2)
ax.invert_yaxis()
plt.get_current_fig_manager().full_screen_toggle()
plt.show()
