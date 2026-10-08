import csv
import json
import os
import random
import time
from pathlib import Path
import joblib
import pandas as pd
import json
import networkx as nx

from task_database import (
    calculate_robot_rtuf,
    get_robot_tasks,
    initialize_database,
    insert_task,
    update_actual_completion_time,
    update_remaining_time,
    update_task_status,
)

from new_warehouse_map import G, nodes

os.chdir(Path(__file__).resolve().parent)
clf_xgb = joblib.load("/Users/minthantkyaw/Documents/Operation Files/YTU/2025-26 Final Year/Python/MILP Dataset Generation with custom guided lines/working/Final Dataset/xgb_classifier.pkl")
print("XGBoost classifier loaded.")

with open("scenario_50.json", "r") as f:
    scenario_tasks = json.load(f)

def get_classifier_feature_names(model):
    if hasattr(model, "feature_names_in_"):
        return list(model.feature_names_in_)

    booster_feature_names = model.get_booster().feature_names

    if booster_feature_names:
        return list(booster_feature_names)

    return [
        "r1_next_x",
        "r1_next_y",
        "r1_tuf",
        "r2_next_x",
        "r2_next_y",
        "r2_tuf",
        "r1_queue_length",
        "r2_queue_length",
        "tuf_difference",
        "task_pl_x",
        "task_pl_y",
        "task_dl_x",
        "task_dl_y",
        "r1_dist_to_pickup",
        "r2_dist_to_pickup",
        "task_dist_to_dropoff",
        "task_type",
        "deadline",
    ]


CLASSIFIER_FEATURE_COLUMNS = get_classifier_feature_names(clf_xgb)
print(f"Classifier feature columns: {CLASSIFIER_FEATURE_COLUMNS}")

dataset_rows = []
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


SIMULATION_FPS = 12.5


MAP_SCALE = 0.0254

PICKUP_NODES = ["Gate_1", "Gate_2"]
DROPOFF_NODES = ["Gate_3", "Gate_4"]

STORAGE_NODES = [
    "Yellow_1", "Yellow_2", "Yellow_3", "Yellow_4", "Yellow_5",
    "Purple_1", "Purple_2", "Purple_3", "Purple_4", "Purple_5",
    "Green_1", "Green_2", "Green_3", "Green_4", "Green_5", "Green_6", "Green_7",
    "Orange_1", "Orange_2", "Orange_3", "Orange_4", "Orange_5", "Orange_6", "Orange_7",
]


def heuristic(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)


def generate_robots():
    return {
        "R1": {
            "node": "Parking_1",
            "next_node": "Parking_1",
            "real_time_until_free": 0.0,
            "queue": [],
            "logical_path": [],
            "wait": 0,
            "has_payload": False,
            "speed": 0.25333,
        },
        "R2": {
            "node": "Parking_2",
            "next_node": "Parking_2",
            "real_time_until_free": 0.0,
            "queue": [],
            "logical_path": [],
            "wait": 0,
            "has_payload": False,
            "speed": 0.25333,
        },
    }


task_id = 0


def compute_travel_time(start_node, end_node, speed, graph, map_scale):
    dist = nx.astar_path_length(
        graph,
        start_node,
        end_node,
        heuristic=heuristic,
        weight="weight",
    )

    return (dist * map_scale) / speed


def compute_task_time_seconds_from_node(start_node, robot, task):
    d1 = nx.astar_path_length(
        G,
        start_node,
        task["PL"],
        heuristic=heuristic,
        weight="weight",
    )

    d2 = nx.astar_path_length(
        G,
        task["PL"],
        task["DL"],
        heuristic=heuristic,
        weight="weight",
    )

    return (
        ((d1 + d2) * MAP_SCALE)
        / robot["speed"]
        + task["pickup_time"]
        + task["dropoff_time"]
    )


def solve_xgboost_fifo(batch_tasks, robots):

    robot_state = snapshot_robot_state(robots)
    temporary_state = {

        robot_name: robot_state[robot_name].copy()

        for robot_name in robot_state

    }

    assignments = {}

    priorities = {}

    for i, task in enumerate(batch_tasks):

        r1_nx, r1_ny = nodes[temporary_state["R1"]["next_node"]]
        r2_nx, r2_ny = nodes[temporary_state["R2"]["next_node"]]

        pl_x, pl_y = nodes[task["PL"]]
        dl_x, dl_y = nodes[task["DL"]]

        r1_dist = (
            nx.astar_path_length(
                G,
                temporary_state["R1"]["next_node"],
                task["PL"],
                heuristic=heuristic,
                weight="weight",
            )
            * MAP_SCALE
        )

        r2_dist = (
            nx.astar_path_length(
                G,
                temporary_state["R2"]["next_node"],
                task["PL"],
                heuristic=heuristic,
                weight="weight",
            )
            * MAP_SCALE
        )

        task_distance = (
            nx.astar_path_length(
                G,
                task["PL"],
                task["DL"],
                heuristic=heuristic,
                weight="weight",
            )
            * MAP_SCALE
        )

        r1_tuf = temporary_state["R1"]["real_time_until_free"]
        r2_tuf = temporary_state["R2"]["real_time_until_free"]

        features = pd.DataFrame([{

            "r1_next_x": r1_nx * MAP_SCALE,
            "r1_next_y": r1_ny * MAP_SCALE,
            "r1_tuf": r1_tuf,

            "r2_next_x": r2_nx * MAP_SCALE,
            "r2_next_y": r2_ny * MAP_SCALE,
            "r2_tuf": r2_tuf,

            "r1_queue_length": temporary_state["R1"]["queue_length"],
            "r2_queue_length": temporary_state["R2"]["queue_length"],

            "tuf_difference": r1_tuf - r2_tuf,

            "task_pl_x": pl_x * MAP_SCALE,
            "task_pl_y": pl_y * MAP_SCALE,

            "task_dl_x": dl_x * MAP_SCALE,
            "task_dl_y": dl_y * MAP_SCALE,

            "r1_dist_to_pickup": r1_dist,
            "r2_dist_to_pickup": r2_dist,

            "task_dist_to_dropoff": task_distance,

            "task_type": task["type"],

            "deadline": task["deadline"]

        }])
        features = features[CLASSIFIER_FEATURE_COLUMNS]
        
        prediction_start = time.perf_counter()

        pred = clf_xgb.predict(features)[0]

        evaluation_metrics["computation_time"] += (
            time.perf_counter() - prediction_start
        )

        robot_name = "R1" if pred == 0 else "R2"

        assignments[i] = robot_name
        assigned = temporary_state[robot_name]

        task_duration = compute_task_time_seconds_from_node(
            assigned["next_node"],
            robots[robot_name],
            task,
        )

        assigned["real_time_until_free"] += task_duration

        assigned["queue_length"] += 1

        assigned["next_node"] = task["DL"]

    #
    # FIFO priorities
    #

    fifo_counter = {

        "R1": 1,
        "R2": 1

    }

    for i in range(len(batch_tasks)):

        robot = assignments[i]

        priorities[i] = fifo_counter[robot]

        fifo_counter[robot] += 1

    return assignments, priorities

def snapshot_robot_state(robots):
    return {
        robot_name: {
            "next_node": robot["next_node"],
            "real_time_until_free": calculate_robot_rtuf(robot_name),
            "queue_length": len(get_robot_tasks(robot_name)),
        }
        for robot_name, robot in robots.items()
    }


def encode_robot_label(robot_name):
    return {
        "R1": 0,
        "R2": 1,
    }[robot_name]


def update_robot_queue_remaining_time(robot_name, robots):
    robot = robots[robot_name]

    if not robot["queue"]:
        robot["real_time_until_free"] = 0.0
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
                weight="weight",
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
                task,
            )

        update_remaining_time(task["id"], remaining_time)
        next_start_node = task["DL"]

    robot["real_time_until_free"] = calculate_robot_rtuf(robot_name)


def apply_batch_assignments(batch_counter, batch_tasks, assignments, priorities, robots):
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

        robot = robots[robot_name]
        next_node = robot["next_node"]

        for task_index in ordered_task_indexes:
            task = batch_tasks[task_index]
            task_duration = compute_task_time_seconds_from_node(
                next_node,
                robot,
                task,
            )

            insert_task(
                task_id=task["id"],
                task_type=task["type"],
                PL=task["PL"],
                DL=task["DL"],
                assigned_robot=robot_name,
                predicted_duration=task_duration,
            )

            task["assigned_robot"] = robot_name
            task["sequence_rank"] = priorities[task_index]
            task["group_id"] = f"B{batch_counter:05d}_{robot_name}"

            robot["queue"].append(task)
            robot["real_time_until_free"] += task_duration
            robot["next_node"] = task["DL"]

            next_node = task["DL"]

def calculate_reordering(batch_tasks, priorities):

    displacement = []

    for task_index in priorities:

        original = task_index + 1
        new = priorities[task_index]

        displacement.append(
            abs(original - new)
        )

    return sum(displacement) / len(displacement)

def start_next_task(robot_name, robots, sim_time):
    robot = robots[robot_name]

    if "current_task" in robot or not robot["queue"]:
        return

    robot["current_task"] = robot["queue"][0]
    robot["current_task"]["actual_start_time"] = sim_time
    robot["has_payload"] = False
    robot["logical_path"] = []

    update_task_status(robot["current_task"]["id"], "IN_PROGRESS")
    update_robot_queue_remaining_time(robot_name, robots)


def move_one_node_toward(robot_name, robots, target_node):
    robot = robots[robot_name]

    if robot["node"] == target_node:
        return

    if not robot["logical_path"]:
        path = nx.astar_path(
            G,
            robot["node"],
            target_node,
            heuristic=heuristic,
            weight="weight",
        )
        robot["logical_path"] = path[1:]

    if robot["logical_path"]:
        robot["node"] = robot["logical_path"].pop(0)
        update_robot_queue_remaining_time(robot_name, robots)


def update_robot_execution(robot_name, robots, sim_time):
    robot = robots[robot_name]

    start_next_task(robot_name, robots, sim_time)

    if "current_task" not in robot:
        return

    if robot["wait"] > 0:
        robot["wait"] -= 1
        return

    task = robot["current_task"]

    if not robot["has_payload"]:
        if robot["node"] != task["PL"]:
            move_one_node_toward(robot_name, robots, task["PL"])
        else:
            robot["wait"] = int(task["pickup_time"] * SIMULATION_FPS)
            robot["has_payload"] = True
            robot["logical_path"] = []
            update_robot_queue_remaining_time(robot_name, robots)

        return

    if robot["node"] != task["DL"]:
        move_one_node_toward(robot_name, robots, task["DL"])
        return

    robot["wait"] = int(task["dropoff_time"] * SIMULATION_FPS)
    update_task_status(task["id"], "COMPLETED")
    update_remaining_time(task["id"], 0)

    robot["queue"].pop(0)
    robot["real_time_until_free"] = calculate_robot_rtuf(robot_name)

    actual_completion_time = sim_time - task["actual_start_time"]
    flow_time = sim_time
    
    lateness = max(0.0, flow_time - task["deadline"])

    if lateness == 0:
        evaluation_counters["successful_tasks"] += 1

    evaluation_counters["completed_tasks"] += 1
    evaluation_metrics["cumulative_penalty"] += lateness

    update_actual_completion_time(task["id"], actual_completion_time)

    del robot["current_task"]
    robot["has_payload"] = False
    robot["logical_path"] = []


def export_dataset(rows, output_path):
    if not rows:
        return

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_progress(frame, batch_counter, rows, robots):
    if frame % 500 != 0:
        return

    r1_tuf = calculate_robot_rtuf("R1")
    r2_tuf = calculate_robot_rtuf("R2")

    print(
        f"frame={frame} "
        f"batches={batch_counter} "
        f"rows={len(rows)} "
        f"R1_tuf={r1_tuf:.2f} "
        f"R2_tuf={r2_tuf:.2f} "
        f"R1_q={len(robots['R1']['queue'])} "
        f"R2_q={len(robots['R2']['queue'])}"
    )


def update_system_metrics(robots):
    return


def finalize_metrics(frame, batch_counter):
    completed = evaluation_counters["completed_tasks"]
    generated = evaluation_counters["total_tasks_generated"]

    evaluation_metrics["makespan"] = round(
        frame / SIMULATION_FPS,
        2,
    )
    evaluation_metrics["task_success_rate"] = (
    round(
        evaluation_counters["successful_tasks"]
        /
        evaluation_counters["total_tasks_generated"]
        * 100,
        2,
    )
        if evaluation_counters["total_tasks_generated"]
        else 0.0
    )
    evaluation_metrics["cumulative_penalty"] = round(
        evaluation_metrics["cumulative_penalty"],
        2,
    )
    evaluation_metrics["computation_time"] = round(
        evaluation_metrics["computation_time"],
        6,
    )


def export_metrics(metrics, output_path):
    with open(output_path, "w") as f:
        json.dump(metrics, f, indent=4)

def export_schedule(batch_tasks, assignments, priorities, output_path):

    schedule = {
        "R1": [],
        "R2": []
    }

    for task_index, robot in assignments.items():

        schedule[robot].append({

            "task_id": batch_tasks[task_index]["id"],

            "priority": priorities[task_index],

            "pickup": batch_tasks[task_index]["PL"],

            "dropoff": batch_tasks[task_index]["DL"],

            "deadline": batch_tasks[task_index]["deadline"]

        })

    for robot in schedule:

        schedule[robot].sort(
            key=lambda x: x["priority"]
        )

    with open(output_path, "w") as f:

        json.dump(schedule, f, indent=4)

def robots_have_work(robots):
    return any(
        robot["queue"] or "current_task" in robot
        for robot in robots.values()
    )


def run_headless_generation():
    initialize_database()

    robots = generate_robots()
    batch_counter = 0
    frame = 0

    started_at = time.perf_counter()
    batch_counter = 1

    batch_tasks = scenario_tasks

    evaluation_counters["total_tasks_generated"] = len(batch_tasks)

    robot_state = snapshot_robot_state(robots)

    assignments, priorities = solve_xgboost_fifo(
        batch_tasks,
        robots,
    )

    print(
        "Average Reordering:",
        calculate_reordering(
            batch_tasks,
            priorities,
        )
    )

    export_schedule(
        batch_tasks,
        assignments,
        priorities,
        "xgboost_schedule.json"
    )

    from collections import Counter

    counts = Counter(assignments.values())

    print(counts)


    apply_batch_assignments(
        batch_counter,
        batch_tasks,
        assignments,
        priorities,
        robots,
    )

    while robots_have_work(robots):
        frame += 1
        sim_time = frame / SIMULATION_FPS

        for robot_name in robots.keys():
            update_robot_execution(robot_name, robots, sim_time)

        update_system_metrics(robots)
        print_progress(frame, batch_counter, dataset_rows, robots)

    finalize_metrics(frame, batch_counter)

    metrics_output_path = (
        f"xgboost_classifier_fifo_metrics.json"
    )
    export_metrics(evaluation_metrics, metrics_output_path)

    elapsed = time.perf_counter() - started_at

    print("\n==============================")
    print("HEADLESS DATASET GENERATION COMPLETE")
    print(f"Saved rows: {len(dataset_rows)}")
    print(f"Metrics: {metrics_output_path}")
    print(f"Wall-clock seconds: {elapsed:.2f}")
    print(f"Makespan: {evaluation_metrics['makespan']:.2f}")
    print(f"Computation Time: {evaluation_metrics['computation_time']:.6f}")
    print(f"Cumulative Penalty: {evaluation_metrics['cumulative_penalty']:.2f}")
    print(f"Task Success Rate: {evaluation_metrics['task_success_rate']:.2f}%")
    print("==============================")

    print("Completed:", evaluation_counters["completed_tasks"])
    print("Generated:", evaluation_counters["total_tasks_generated"])


if __name__ == "__main__":
    run_headless_generation()
