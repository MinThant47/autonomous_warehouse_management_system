import networkx as nx
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import gurobipy as gp
from gurobipy import GRB
import numpy as np

# -----------------------------
# CONFIG 🔥
# -----------------------------
NUM_ROBOTS = 10
FRAMES_PER_EDGE = 10

# -----------------------------
# GRID
# -----------------------------
G = nx.Graph()
size = 4

x_coords = [0]
y_coords = [0]

for i in range(1, size):
    x_coords.append(x_coords[-1] + random.uniform(3, 8))
    y_coords.append(y_coords[-1] + random.uniform(3, 8))

nodes = {}
for i in range(size):
    for j in range(size):
        name = f"N{i}{j}"
        nodes[name] = (x_coords[i], y_coords[j])
        G.add_node(name, pos=nodes[name])

def manhattan_dist(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

for i in range(size):
    for j in range(size):
        if i < size - 1:
            G.add_edge(f"N{i}{j}", f"N{i+1}{j}", weight=manhattan_dist(f"N{i}{j}", f"N{i+1}{j}"))
        if j < size - 1:
            G.add_edge(f"N{i}{j}", f"N{i}{j+1}", weight=manhattan_dist(f"N{i}{j}", f"N{i}{j+1}"))

def heuristic(a, b):
    return manhattan_dist(a, b)

# -----------------------------
# ROBOTS (SCALABLE)
# -----------------------------
def generate_robots(n):
    node_list = list(G.nodes)
    robots = {}

    for i in range(n):
        node = random.choice(node_list)

        robots[f"R{i}"] = {
            "pos": nodes[node],
            "node": node,
            "next_node": node,
            "time_until_free": 0,
            "queue": [],
            "path": [],
            "wait": 0,
            "speed": random.uniform(0.4, 0.6)
        }

    return robots

# -----------------------------
# TASK
# -----------------------------
task_id = 0
def generate_task():
    global task_id
    task_id += 1

    node_list = list(G.nodes)
    pl = random.choice(node_list)
    dl = random.choice(node_list)

    while dl == pl:
        dl = random.choice(node_list)

    return {
        "id": task_id,
        "PL": pl,
        "DL": dl,
        "pickup_time": random.uniform(2, 4),
        "dropoff_time": random.uniform(2, 4),
        "deadline": random.uniform(40, 100)
    }

# -----------------------------
# TA COST (SYNCED)
# -----------------------------
def compute_total_time(robot, task):

    start = robot["next_node"]

    d1 = nx.astar_path_length(G, start, task["PL"], heuristic=heuristic)
    d2 = nx.astar_path_length(G, task["PL"], task["DL"], heuristic=heuristic)

    d1_frames = d1 * FRAMES_PER_EDGE
    d2_frames = d2 * FRAMES_PER_EDGE

    handling_frames = int((task["pickup_time"] + task["dropoff_time"]) * FRAMES_PER_EDGE)

    return robot["time_until_free"] + d1_frames + d2_frames + handling_frames

# -----------------------------
# MILP (SCALABLE)
# -----------------------------
def solve_milp(pt, deadline):

    model = gp.Model()
    model.setParam("OutputFlag", 0)

    K = range(len(pt))

    X = model.addVars(K, vtype=GRB.BINARY)
    Cmax = model.addVar()
    L = model.addVar()

    model.addConstr(sum(X[k] for k in K) == 1)
    model.addConstr(sum(X[k]*pt[k] for k in K) <= Cmax)
    model.addConstr(sum(X[k]*pt[k] for k in K) <= deadline + L)
    model.addConstr(L >= 0)

    model.setObjective(Cmax + 10 * L, GRB.MINIMIZE)
    model.optimize()

    for k in K:
        if X[k].x > 0.5:
            return k

# -----------------------------
# PATH
# -----------------------------
def smooth_path(path):
    coords = [nodes[n] for n in path]
    smooth = []

    for i in range(len(coords)-1):
        x1, y1 = coords[i]
        x2, y2 = coords[i+1]

        for t in np.linspace(0, 1, FRAMES_PER_EDGE):
            smooth.append((x1+(x2-x1)*t, y1+(y2-y1)*t))

    return smooth

# -----------------------------
# INIT
# -----------------------------
robots = generate_robots(NUM_ROBOTS)
robot_names = list(robots.keys())

task_timer = 0
current_task_visual = None

# color palette
colors = plt.cm.tab10(np.linspace(0, 1, NUM_ROBOTS))
robot_dots = {}

# -----------------------------
# PLOT
# -----------------------------
fig, ax = plt.subplots(figsize=(7,7))
pos = nx.get_node_attributes(G, 'pos')
nx.draw(G, pos, ax=ax, node_color='lightgray', with_labels=True)

for i, name in enumerate(robot_names):
    dot, = ax.plot([], [], 'o', color=colors[i], markersize=8)
    robot_dots[name] = dot

task_pick = ax.scatter([], [], c='orange', s=200)
task_drop = ax.scatter([], [], c='blue', s=200)

# -----------------------------
# UPDATE LOOP
# -----------------------------
def update(frame):
    global task_timer, current_task_visual

    task_timer += 1

    # 🔥 NEW TASK
    if task_timer > random.randint(10, 20):
        task = generate_task()
        current_task_visual = task

        pt = []
        for name in robot_names:
            t = compute_total_time(robots[name], task)
            pt.append(t)

        winner_idx = solve_milp(pt, task["deadline"])
        winner = robot_names[winner_idx]

        robot = robots[winner]

        robot["time_until_free"] = pt[winner_idx]
        robot["next_node"] = task["DL"]
        robot["queue"].append(task)

        print(f"Task {task['id']} → {winner}")
        task_timer = 0

    # 🔥 EXECUTION
    for name, robot in robots.items():

        # decrease workload
        if robot["time_until_free"] > 0:
            robot["time_until_free"] = max(0, robot["time_until_free"] - 1)

        if robot["wait"] > 0:
            robot["wait"] -= 1
            continue

        if "current_task" not in robot and robot["queue"]:
            robot["current_task"] = robot["queue"][0]
            robot["has_payload"] = False

        if "current_task" in robot:
            task = robot["current_task"]

            # GO TO PICKUP
            if not robot["has_payload"]:

                if robot["node"] != task["PL"]:

                    if not robot["path"]:
                        path = nx.astar_path(G, robot["node"], task["PL"], heuristic=heuristic)
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
                        path = nx.astar_path(G, task["PL"], task["DL"], heuristic=heuristic)
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
    for name in robot_names:
        robot_dots[name].set_data(
            [robots[name]["pos"][0]],
            [robots[name]["pos"][1]]
        )

    if current_task_visual:
        pl = nodes[current_task_visual["PL"]]
        dl = nodes[current_task_visual["DL"]]

        task_pick.set_offsets([pl])
        task_drop.set_offsets([dl])

    return list(robot_dots.values()) + [task_pick, task_drop]

# -----------------------------
# RUN
# -----------------------------
ani = animation.FuncAnimation(fig, update, interval=50)
plt.show()