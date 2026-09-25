import networkx as nx
import numpy as np
import joblib
import pandas as pd
from datetime import datetime, timezone

# 1 graph distance unit = 0.0254 meter (1 inch)
MAP_SCALE = 0.0254
DEFAULT_ROBOT_SPEED = 0.25333
# -----------------------------

classifier = joblib.load(
    "models/xgb_classifier_deadline.pkl"
)

ranker = joblib.load(
    "models/xgb_ranker.pkl"
)

# -----------------------------
# WAREHOUSE GRAPH 
# -----------------------------
from new_warehouse_map import G, nodes 

def heuristic(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

# -----------------------------
# ROBOTS
# -----------------------------
def generate_robots():
    return {
        "R1": {
            "node": "Parking_1",
            "next_node": "Parking_1",
            "queue": [],
            "manual_queue_order": False,
            "speed": DEFAULT_ROBOT_SPEED,
            "status": "IDLE",
            "has_payload": False,
            "last_node_update": None,
            "last_update_source": None,
        },
        "R2": {
            "node": "Parking_2",
            "next_node": "Parking_2",
            "queue": [],
            "manual_queue_order": False,
            "speed": DEFAULT_ROBOT_SPEED,
            "status": "IDLE",
            "has_payload": False,
            "last_node_update": None,
            "last_update_source": None,
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

    # A UI reorder takes precedence over the learned ranker until the queue
    # drains. New work is appended, and the chosen order is retained.
    if robot.get("manual_queue_order"):
        for rank, task in enumerate(robot["queue"], start=1):
            task["sequence_rank"] = rank
        for position, task in enumerate(pending_tasks, start=1):
            task["pending_rank"] = position
            task["previous_pending_rank"] = position
            task["rank_move"] = "same"
            task["rank_delta"] = 0
        update_current_task_remaining_time(robot_name)
        robot["next_node"] = robot["queue"][-1]["DL"] if robot["queue"] else robot["node"]
        return

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

    scores = ranker.predict(
        ranker_features
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

def dispatch_task(task):
    """Assign one server task and rerank the selected robot's pending queue.

    ``task`` must contain: id, type, PL, DL, pickup_time, dropoff_time, deadline.
    PL/DL must be node names in ``new_warehouse_map.G``.
    """
    validate_task(task)
    features = build_classifier_features(task, snapshot_robot_state())
    prediction = classifier.predict(features)[0]
    robot_name = "R1" if prediction == 0 else "R2"
    robot = robots[robot_name]
    robot_was_free = not robot["queue"]

    task = dict(task)  # do not mutate the server request object
    task["assigned_robot"] = robot_name
    task["status"] = "QUEUED"
    robot["queue"].append(task)

    # The first task is fixed once the robot is available to execute it.
    # ``rerank_robot_queue`` already treats this fixed head separately, so only
    # later queued tasks may be reordered by the ranker.
    if robot_was_free:
        start_next_task(robot_name)

    rerank_robot_queue(robot_name)
    refresh_robot_projection(robot_name)

    return {
        "assigned_robot": robot_name,
        "queue": task_queue(robot_name),
        "robot_state": robot_state(robot_name),
    }


def reorder_pending_tasks(robot_name, ordered_task_ids):
    """Apply an explicit pending task order while keeping the active head fixed."""
    if robot_name not in robots:
        raise ValueError(f"Unknown robot ID: {robot_name}")
    robot = robots[robot_name]
    fixed_prefix = [robot["queue"][0]] if "current_task" in robot and robot["queue"] else []
    pending = robot["queue"][len(fixed_prefix):]
    pending_ids = [task["id"] for task in pending]
    if (not isinstance(ordered_task_ids, list)
            or any(isinstance(task_id, bool) or not isinstance(task_id, int) for task_id in ordered_task_ids)
            or len(ordered_task_ids) != len(set(ordered_task_ids))
            or set(ordered_task_ids) != set(pending_ids)):
        raise ValueError("ordered_task_ids must contain every pending task ID exactly once")

    by_id = {task["id"]: task for task in pending}
    robot["queue"] = fixed_prefix + [by_id[task_id] for task_id in ordered_task_ids]
    robot["manual_queue_order"] = bool(pending)
    for rank, task in enumerate(robot["queue"], start=1):
        task["sequence_rank"] = rank
    for position, task in enumerate(robot["queue"][len(fixed_prefix):], start=1):
        task["pending_rank"] = position
        task["previous_pending_rank"] = position
        task["rank_move"] = "manual"
        task["rank_delta"] = 0
    update_current_task_remaining_time(robot_name)
    robot["next_node"] = robot["queue"][-1]["DL"] if robot["queue"] else robot["node"]
    refresh_robot_projection(robot_name)
    return robot_state(robot_name)
    
# -----------------------------
# TA COST (SYNCED)
# -----------------------------
# -----------------------------
# REAL PHYSICAL TIME
# -----------------------------

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
    # The WMS reports the current phase and node; service time is included
    # only while the robot reports that it is at the relevant station.
    wait_time = 0.0

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


# ---------------------------------------------------------------------------
# WMS / ROBOT INTEGRATION API
# ---------------------------------------------------------------------------
robots = generate_robots()


def update_current_task_remaining_time(robot_name):
    """Update in-memory projected remaining time; no simulation database is used."""
    robot = robots[robot_name]
    if not robot["queue"]:
        return 0.0
    remaining = evaluate_fixed_robot_queue(robot_name)
    robot["queue"][0]["estimated_remaining_seconds"] = remaining
    return remaining


def validate_task(task):
    required = {"id", "type", "PL", "DL", "pickup_time", "dropoff_time", "deadline"}
    missing = required - set(task)
    if missing:
        raise ValueError(f"Task is missing required fields: {sorted(missing)}")
    unknown_nodes = {task["PL"], task["DL"]} - set(G.nodes)
    if unknown_nodes:
        raise ValueError(f"Task contains unknown map nodes: {sorted(unknown_nodes)}")

def task_queue(robot_name):
    """Return the current queue in dispatch order, safe to serialize as JSON."""
    return [dict(task) for task in robots[robot_name]["queue"]]


def robot_state(robot_name):
    """Return the scheduler state that should be exposed to the WMS."""
    robot = robots[robot_name]
    return {
        "robot_id": robot_name,
        "node": robot["node"],
        "status": robot["status"],
        "has_payload": robot["has_payload"],
        "speed_mps": robot["speed"],
        "current_task_id": robot.get("current_task", {}).get("id"),
        "queue_length": len(robot["queue"]),
        "estimated_seconds_until_free": evaluate_fixed_robot_queue(robot_name),
        "last_node_update": robot["last_node_update"],
        "last_update_source": robot["last_update_source"],
        "queue": task_queue(robot_name),
    }


def update_robot_node(robot_name, node_id, source="mqtt"):
    """Record a robot's reported map node and refresh its XGBoost inputs.

    The physical robot only needs to report a node name from ``new_warehouse_map``.
    Task routing remains external. With node-only reports, reaching the task's
    pickup is treated as pickup complete and reaching its dropoff as complete.
    """
    if robot_name not in robots:
        raise ValueError(f"Unknown robot ID: {robot_name}")
    if node_id not in G.nodes:
        raise ValueError(f"Unknown map node: {node_id}")

    robot = robots[robot_name]
    robot["node"] = node_id
    robot["last_node_update"] = datetime.now(timezone.utc).isoformat()
    robot["last_update_source"] = source

    current_task = robot.get("current_task")
    if current_task and current_task["status"] == "TO_PICKUP" and node_id == current_task["PL"]:
        current_task["status"] = "TO_DROPOFF"
        robot["has_payload"] = True
        robot["status"] = "TO_DROPOFF"
    elif current_task and current_task["status"] == "TO_DROPOFF" and node_id == current_task["DL"]:
        complete_current_task(robot_name)
    elif current_task:
        robot["status"] = current_task["status"]
    else:
        robot["status"] = "IDLE"

    refresh_robot_projection(robot_name)
    return robot_state(robot_name)


def start_next_task(robot_name):
    """Make the queue head active without allowing the ranker to move it."""
    robot = robots[robot_name]
    if not robot["queue"]:
        robot.pop("current_task", None)
        robot["status"] = "IDLE"
        robot["has_payload"] = False
        return None

    task = robot["queue"][0]
    task["status"] = "TO_PICKUP"
    task["sequence_rank"] = 1
    task["pending_rank"] = None
    robot["current_task"] = task
    robot["status"] = "TO_PICKUP"
    robot["has_payload"] = False
    return task


def complete_current_task(robot_name):
    """Remove the completed fixed task and activate the next ranked task."""
    robot = robots[robot_name]
    current_task = robot.get("current_task")
    if not current_task:
        return None

    current_task["status"] = "COMPLETED"
    if robot["queue"] and robot["queue"][0]["id"] == current_task["id"]:
        robot["queue"].pop(0)
    else:
        robot["queue"] = [task for task in robot["queue"] if task["id"] != current_task["id"]]
    robot.pop("current_task", None)
    robot["has_payload"] = False
    start_next_task(robot_name)
    rerank_robot_queue(robot_name)
    if not robot["queue"]:
        robot["manual_queue_order"] = False
    return current_task


def refresh_robot_projection(robot_name):
    """Recalculate the queue's projected tail node and completion time."""
    robot = robots[robot_name]
    robot["next_node"] = robot["queue"][-1]["DL"] if robot["queue"] else robot["node"]
    for task in robot["queue"]:
        task["estimated_remaining_seconds"] = evaluate_fixed_robot_queue(robot_name)
