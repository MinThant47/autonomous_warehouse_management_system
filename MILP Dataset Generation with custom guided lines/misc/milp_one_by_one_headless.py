import csv
import os
import json
import random
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import gurobipy as gp
import networkx as nx
import numpy as np
import pandas as pd
from gurobipy import GRB

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
    "successful_tasks": 0
}

TARGET_DATASET_SIZE = 4000

BATCH_INTERVAL_SECONDS = 3.44

SIMULATION_FPS = 12.5

BATCH_INTERVAL_FRAMES = BATCH_INTERVAL_SECONDS * SIMULATION_FPS

MAP_SCALE = 0.0254
DEFAULT_ROBOT_SPEED = 0.25333

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


def generate_task():
    global task_id
    task_id += 1

    task_type = random.choices([1, 2, 3], weights=[0.4, 0.4, 0.2])[0]

    if task_type == 1:
        pl = random.choice(PICKUP_NODES)
        dl = random.choice(STORAGE_NODES)
    elif task_type == 2:
        pl = random.choice(STORAGE_NODES)
        dl = random.choice(DROPOFF_NODES)
    else:
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
        "deadline": 300,
    }


def compute_travel_time(start_node, end_node, speed, graph, map_scale):
    dist = nx.astar_path_length(
        graph,
        start_node,
        end_node,
        heuristic=heuristic,
        weight="weight",
    )

    return (dist * map_scale) / speed


def graph_distance_m(start_node, end_node):
    return (
        nx.astar_path_length(
            G,
            start_node,
            end_node,
            heuristic=heuristic,
            weight="weight",
        )
        * MAP_SCALE
    )


def graph_travel_time_s(start_node, end_node, speed=DEFAULT_ROBOT_SPEED):
    return graph_distance_m(start_node, end_node) / speed


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


def compute_current_task_remaining_time(robot):
    if "current_task" not in robot:
        return 0.0

    task = robot["current_task"]
    wait_time = robot["wait"] / SIMULATION_FPS

    if robot["has_payload"]:
        if robot["node"] == task["DL"]:
            return wait_time

        d = nx.astar_path_length(
            G,
            robot["node"],
            task["DL"],
            heuristic=heuristic,
            weight="weight",
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
            weight="weight",
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
        wait_time
        + ((d1 + d2) * MAP_SCALE) / robot["speed"]
        + task["pickup_time"]
        + task["dropoff_time"]
    )


def get_frozen_and_pending_tasks(robot):
    if "current_task" in robot and robot["queue"]:
        return [robot["queue"][0]], robot["queue"][1:]

    return [], list(robot["queue"])


def get_online_sequence_base(robot):
    if "current_task" in robot and robot["queue"]:
        return compute_current_task_remaining_time(robot), robot["queue"][0]["DL"]

    return 0.0, robot["node"]


def evaluate_fixed_robot_queue(robot_name, robots, sim_time):
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
            - current_task["deadline"],
        )

    next_start_node = start_node

    for task in pending_tasks:
        task_time = compute_task_time_seconds_from_node(
            next_start_node,
            robot,
            task,
        )

        completion_from_now += task_time
        created_time = task.get("created_sim_time", sim_time)
        cumulative_lateness += max(
            0.0,
            sim_time
            + completion_from_now
            - created_time
            - task["deadline"],
        )
        next_start_node = task["DL"]

    return completion_from_now, cumulative_lateness


def solve_pending_sequence_for_robot(
    robot_name,
    pending_tasks,
    robots,
    sim_time,
    other_robot_completion,
    other_robot_lateness,
):
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
            - frozen_task["deadline"],
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
            MAP_SCALE,
        )

        task_duration[t] = (
            compute_travel_time(
                pending_tasks[t]["PL"],
                pending_tasks[t]["DL"],
                robot["speed"],
                G,
                MAP_SCALE,
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
                MAP_SCALE,
            )

    Y = model.addVars(T, T, vtype=GRB.BINARY, name="Sequence")
    Start = model.addVars(T, vtype=GRB.CONTINUOUS, name="StartTime")
    End = model.addVars(T, vtype=GRB.CONTINUOUS, name="EndTime")
    Lateness = model.addVars(T, vtype=GRB.CONTINUOUS, name="Lateness")
    Cmax = model.addVar(vtype=GRB.CONTINUOUS, name="Cmax")

    for t in T:
        model.addConstr(Start[t] >= base_time + travel_init[t])
        model.addConstr(End[t] >= Start[t] + task_duration[t])
        model.addConstr(Cmax >= End[t])
        model.addConstr(
            Lateness[t]
            >=
            sim_time
            + End[t]
            - pending_tasks[t].get("created_sim_time", sim_time)
            - pending_tasks[t]["deadline"]
        )
        model.addConstr(Lateness[t] >= 0)

    model.addConstr(Cmax >= other_robot_completion)

    for t1 in T:
        for t2 in T:
            if t1 < t2:
                model.addConstr(Y[t1, t2] + Y[t2, t1] == 1)

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
        + 10
        * (
            other_robot_lateness
            + selected_robot_frozen_lateness
            + sum(Lateness[t] for t in T)
        ),
        GRB.MINIMIZE,
    )

    optimization_start = time.perf_counter()
    model.optimize()
    evaluation_metrics["computation_time"] += (
        time.perf_counter() - optimization_start
    )

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
    ordered_items.sort(key=lambda item: item["start_time"])

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
        Cmax.x,
    )


def solve_online_milp_arrival(task, robots, sim_time):
    task["created_sim_time"] = sim_time

    before_state = snapshot_robot_state(robots)
    candidates = {}

    for robot_name in robots.keys():
        other_robot_name = "R2" if robot_name == "R1" else "R1"

        frozen_tasks, pending_tasks = get_frozen_and_pending_tasks(
            robots[robot_name]
        )
        candidate_pending_tasks = pending_tasks + [task]

        other_completion, other_lateness = evaluate_fixed_robot_queue(
            other_robot_name,
            robots,
            sim_time,
        )

        ordered_pending, objective_value, lateness, cmax = (
            solve_pending_sequence_for_robot(
                robot_name,
                candidate_pending_tasks,
                robots,
                sim_time,
                other_completion,
                other_lateness,
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
        key=lambda robot_name: candidates[robot_name]["objective_value"],
    )

    return selected_robot, candidates[selected_robot], before_state


def apply_online_milp_decision(batch_counter, task, robot_name, decision, robots):
    robot = robots[robot_name]
    task_start_node = (
        robot["queue"][-1]["DL"]
        if robot["queue"]
        else robot["node"]
    )

    task_duration = compute_task_time_seconds_from_node(
        task_start_node,
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
    task["group_id"] = f"B{batch_counter:05d}_{robot_name}"

    robot["queue"] = decision["frozen_tasks"] + decision["ordered_pending"]

    for rank, queued_task in enumerate(robot["queue"], start=1):
        queued_task["sequence_rank"] = rank

    update_robot_queue_remaining_time(robot_name, robots)
    robot["next_node"] = robot["queue"][-1]["DL"] if robot["queue"] else robot["node"]


def solve_milp_batch_joint(tasks, robots, graph, map_scale):
    model = gp.Model()
    model.setParam("OutputFlag", 0)
    model.setParam("TimeLimit", 3)

    T = range(len(tasks))
    R = range(len(robots))
    robot_names = list(robots.keys())
    M = 10000

    travel_init = {}

    for t in T:
        for r in R:
            robot_name = robot_names[r]
            travel_init[t, r] = compute_travel_time(
                robots[robot_name]["next_node"],
                tasks[t]["PL"],
                robots[robot_name]["speed"],
                graph,
                map_scale,
            )

    travel_between = {}

    for t1 in T:
        for t2 in T:
            if t1 != t2:
                for r in R:
                    robot_name = robot_names[r]
                    travel_between[t1, t2, r] = compute_travel_time(
                        tasks[t1]["DL"],
                        tasks[t2]["PL"],
                        robots[robot_name]["speed"],
                        graph,
                        map_scale,
                    )

    task_duration = {}

    for t in T:
        for r in R:
            robot_name = robot_names[r]
            travel_task = compute_travel_time(
                tasks[t]["PL"],
                tasks[t]["DL"],
                robots[robot_name]["speed"],
                graph,
                map_scale,
            )

            task_duration[t, r] = (
                travel_task
                + tasks[t]["pickup_time"]
                + tasks[t]["dropoff_time"]
            )

    X = model.addVars(T, R, vtype=GRB.BINARY, name="Assign")
    Y = model.addVars(T, T, R, vtype=GRB.BINARY, name="Sequence")
    Start = model.addVars(T, vtype=GRB.CONTINUOUS, name="StartTime")
    End = model.addVars(T, vtype=GRB.CONTINUOUS, name="EndTime")
    Lateness = model.addVars(T, vtype=GRB.CONTINUOUS, name="Lateness")
    Cmax = model.addVar(vtype=GRB.CONTINUOUS, name="Cmax")

    for t in T:
        model.addConstr(sum(X[t, r] for r in R) == 1)

    for t in T:
        for r in R:
            model.addConstr(
                End[t]
                >=
                Start[t]
                + task_duration[t, r]
                - M * (1 - X[t, r])
            )

    for t in T:
        for r in R:
            robot_name = robot_names[r]
            rtuf = robots[robot_name]["real_time_until_free"]

            model.addConstr(
                Start[t]
                >=
                rtuf
                + travel_init[t, r]
                - M * (1 - X[t, r])
            )

    for t1 in T:
        for t2 in T:
            if t1 < t2:
                for r in R:
                    model.addConstr(
                        Y[t1, t2, r]
                        + Y[t2, t1, r]
                        >=
                        X[t1, r]
                        + X[t2, r]
                        - 1
                    )

                    model.addConstr(
                        Y[t1, t2, r]
                        + Y[t2, t1, r]
                        <= 1
                    )

                    model.addConstr(Y[t1, t2, r] <= X[t1, r])
                    model.addConstr(Y[t1, t2, r] <= X[t2, r])
                    model.addConstr(Y[t2, t1, r] <= X[t1, r])
                    model.addConstr(Y[t2, t1, r] <= X[t2, r])

    for t1 in T:
        for t2 in T:
            if t1 != t2:
                for r in R:
                    model.addConstr(
                        Start[t2]
                        >=
                        End[t1]
                        + travel_between[t1, t2, r]
                        - M * (1 - Y[t1, t2, r])
                    )

    for t in T:
        model.addConstr(Cmax >= End[t])
        model.addConstr(End[t] <= tasks[t]["deadline"] + Lateness[t])

    model.setObjective(
        Cmax + 10 * sum(Lateness[t] for t in T),
        GRB.MINIMIZE,
    )

    optimization_start = time.perf_counter()

    model.optimize()

    evaluation_metrics["computation_time"] += (
    time.perf_counter() - optimization_start
)

    if model.SolCount == 0:
        raise RuntimeError(
            f"Batch MILP could not find a usable solution. Status: {model.Status}"
        )

    robot_queues = {
        robot_name: []
        for robot_name in robot_names
    }

    for t in T:
        for r in R:
            if X[t, r].x > 0.5:
                robot_queues[robot_names[r]].append({
                    "task_id": t,
                    "start_time": Start[t].x,
                })

    assignments = {}
    priorities = {}

    for robot_name, queue in robot_queues.items():
        queue.sort(key=lambda x: x["start_time"])

        for rank, item in enumerate(queue):
            task_index = item["task_id"]
            assignments[task_index] = robot_name
            priorities[task_index] = rank + 1

    return assignments, priorities


def snapshot_robot_state(robots):
    return {
        robot_name: {
            "next_node": robot["queue"][-1]["DL"] if robot["queue"] else robot["node"],
            "real_time_until_free": evaluate_fixed_robot_queue(
                robot_name,
                robots,
                0.0,
            )[0],
            "queue_length": len(robot["queue"]),
        }
        for robot_name, robot in robots.items()
    }


def encode_robot_label(robot_name):
    return {
        "R1": 0,
        "R2": 1,
    }[robot_name]


def build_batch_dataset_row(
    batch_id,
    batch_position,
    batch_size,
    task,
    robot_state,
    assigned_robot,
    sequence_rank,
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
        weight="weight",
    )

    r2_dist_to_pickup = nx.astar_path_length(
        G,
        robot_state["R2"]["next_node"],
        task["PL"],
        heuristic=heuristic,
        weight="weight",
    )

    task_dist_to_dropoff = nx.astar_path_length(
        G,
        task["PL"],
        task["DL"],
        heuristic=heuristic,
        weight="weight",
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
        "sequence_rank": sequence_rank,
    }


def update_robot_queue_remaining_time(robot_name, robots):
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
                task,
            )

        update_remaining_time(task["id"], remaining_time)
        next_start_node = task["DL"]

    robot["real_time_until_free"] = calculate_robot_rtuf(robot_name)
    robot["next_node"] = robot["queue"][-1]["DL"]


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
    flow_time = sim_time - task["created_sim_time"]

    lateness = max(
        0.0,
        flow_time - task["deadline"]
    )

    if lateness == 0:
        evaluation_counters["successful_tasks"] += 1

    evaluation_counters["completed_tasks"] += 1
    evaluation_metrics["cumulative_penalty"] += lateness

    update_actual_completion_time(task["id"], actual_completion_time)

    del robot["current_task"]
    robot["has_payload"] = False
    robot["logical_path"] = []


def add_ranker_context_features(df):
    if df.empty:
        return df

    df = df.copy()

    df["group_size"] = (
        df.groupby("group_id")["task_id"]
        .transform("count")
    )

    avg_pickup_distances = []
    nearest_pickup_distances = []
    avg_dropoff_to_pickup_distances = []
    nearest_dropoff_to_pickup_distances = []

    for _, group in df.groupby("group_id", sort=False):
        group_records = group.to_dict("records")
        group_size = len(group_records)

        for i, current in enumerate(group_records):
            pickup_dists = []
            dropoff_to_pickup_dists = []

            for j, other in enumerate(group_records):
                if i == j:
                    continue

                pickup_dists.append(
                    graph_distance_m(
                        current["task_pl_node"],
                        other["task_pl_node"],
                    )
                )

                dropoff_to_pickup_dists.append(
                    graph_distance_m(
                        current["task_dl_node"],
                        other["task_pl_node"],
                    )
                )

            if group_size > 1:
                avg_pickup_distances.append(float(np.mean(pickup_dists)))
                nearest_pickup_distances.append(float(np.min(pickup_dists)))
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
        df["r2_tuf"],
    )
    df["other_robot_tuf"] = np.where(
        assigned_is_r1,
        df["r2_tuf"],
        df["r1_tuf"],
    )
    df["assigned_robot_queue_length"] = np.where(
        assigned_is_r1,
        df["r1_queue_length"],
        df["r2_queue_length"],
    )
    df["other_robot_queue_length"] = np.where(
        assigned_is_r1,
        df["r2_queue_length"],
        df["r1_queue_length"],
    )
    df["assigned_robot_dist_to_pickup"] = np.where(
        assigned_is_r1,
        df["r1_dist_to_pickup"],
        df["r2_dist_to_pickup"],
    )
    df["other_robot_dist_to_pickup"] = np.where(
        assigned_is_r1,
        df["r2_dist_to_pickup"],
        df["r1_dist_to_pickup"],
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

    min_slack = (
        df.groupby("group_id")["deadline_margin"]
        .transform("min")
    )
    df["deadline_margin_difference"] = (
        df["deadline_margin"]
        - min_slack
    )
    df["estimated_lateness_if_next"] = np.maximum(
        0.0,
        -df["deadline_margin"],
    )
    df["deadline_margin_ratio"] = np.where(
        df["deadline"] > 0,
        df["deadline_margin"] / df["deadline"],
        0.0,
    )
    df["urgency_per_meter"] = np.where(
        df["total_direct_task_distance"] > 0,
        df["deadline"] / df["total_direct_task_distance"],
        df["deadline"],
    )
    df["queue_context_load"] = (
        df["assigned_robot_tuf"]
        + df["assigned_robot_queue_length"]
    )

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

    for _, group in df.groupby("group_id", sort=False):
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
                    other["task_pl_node"],
                )
                predecessor_distance = graph_distance_m(
                    other["task_dl_node"],
                    current["task_pl_node"],
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
                average_successor_time
                * max(0, group_size - 1)
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


def export_dataset(rows, output_path):
    if not rows:
        return

    df = add_ranker_context_features(
        pd.DataFrame(rows)
    )

    df.to_csv(output_path, index=False)


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

def finalize_metrics(frame):
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

def robots_have_work(robots):
    return any(
        robot["queue"] or "current_task" in robot
        for robot in robots.values()
    )

def run_headless_generation():
    initialize_database()

    robots = generate_robots()
    task_counter = 0
    batch_counter = 0
    frame = 0

    started_at = time.perf_counter()

    while len(dataset_rows) < TARGET_DATASET_SIZE:
        frame += 1
        task_counter += 1
        sim_time = frame / SIMULATION_FPS

        if task_counter >= BATCH_INTERVAL_FRAMES:
            task_counter = 0

            batch_counter += 1

            task = generate_task()
            evaluation_counters["total_tasks_generated"] += 1

            selected_robot, decision, robot_state = solve_online_milp_arrival(
                task,
                robots,
                sim_time,
            )

            apply_online_milp_decision(
                batch_counter,
                task,
                selected_robot,
                decision,
                robots,
            )

            pending_group = decision["ordered_pending"]
            pending_group_size = len(pending_group)

            for pending_position, pending_task in enumerate(
                pending_group,
                start=1,
            ):
                dataset_rows.append(
                    build_batch_dataset_row(
                        batch_counter,
                        pending_position,
                        pending_group_size,
                        pending_task,
                        robot_state,
                        selected_robot,
                        pending_position,
                    )
                )

        for robot_name in robots.keys():
            update_robot_execution(robot_name, robots, sim_time)

        print_progress(frame, batch_counter, dataset_rows, robots)

    # Finish executing all remaining queued tasks
    while robots_have_work(robots):
        frame += 1
        sim_time = frame / SIMULATION_FPS

        for robot_name in robots.keys():
            update_robot_execution(robot_name, robots, sim_time)

        print_progress(
            frame,
            batch_counter,
            dataset_rows,
            robots,
        )
        
    output_path = (
        f"{TARGET_DATASET_SIZE} online one by one headless "
        "warehouse_allocation_sequence_xgboost_dataset.csv"
    )
    export_dataset(dataset_rows, output_path)
    finalize_metrics(frame)

    metrics_output_path = (
        f"{TARGET_DATASET_SIZE} online_one_by_one_milp_metrics.json"
    )

    export_metrics(
        evaluation_metrics,
        metrics_output_path,
    )

    elapsed = time.perf_counter() - started_at

    print("\n==============================")
    print("HEADLESS DATASET GENERATION COMPLETE")
    print(f"Saved rows: {len(dataset_rows)}")
    print(f"Output: {output_path}")
    print(f"Task arrivals: {batch_counter}")
    print(f"Simulated seconds: {frame / SIMULATION_FPS:.2f}")
    print(f"Wall-clock seconds: {elapsed:.2f}")
    print("==============================")

    print(f"Metrics: {metrics_output_path}")

    print(f"Makespan: {evaluation_metrics['makespan']:.2f}")
    print(f"Computation Time: {evaluation_metrics['computation_time']:.6f}")
    print(f"Cumulative Penalty: {evaluation_metrics['cumulative_penalty']:.2f}")
    print(f"Task Success Rate: {evaluation_metrics['task_success_rate']:.2f}%")

    print("==============================")


if __name__ == "__main__":
    run_headless_generation()
