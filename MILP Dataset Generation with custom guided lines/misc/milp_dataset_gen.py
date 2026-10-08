import networkx as nx
import math
import random
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB

# -----------------------------
# 1. CREATE UNEVEN GRID (MANHATTAN SAFE)
# -----------------------------
G = nx.Graph()
size = 4

# Uneven spacing
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

# Manhattan distance
def manhattan_dist(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

# Connect grid (NO diagonals)
for i in range(size):
    for j in range(size):
        if i < size - 1:
            G.add_edge(f"N{i}{j}", f"N{i+1}{j}",
                       weight=manhattan_dist(f"N{i}{j}", f"N{i+1}{j}"))
        if j < size - 1:
            G.add_edge(f"N{i}{j}", f"N{i}{j+1}",
                       weight=manhattan_dist(f"N{i}{j}", f"N{i}{j+1}"))

# -----------------------------
# 2. A* HEURISTIC (MANHATTAN)
# -----------------------------
def heuristic(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

# -----------------------------
# 3. RANDOM ROBOTS
# -----------------------------
def generate_robots():
    node_list = list(G.nodes)

    r1 = random.choice(node_list)
    r2 = random.choice(node_list)

    while r2 == r1:
        r2 = random.choice(node_list)

    return {
        "R1": {"start": r1, "speed": random.uniform(0.4, 0.6)},
        "R2": {"start": r2, "speed": random.uniform(0.4, 0.6)}
    }

# -----------------------------
# 4. TASK GENERATOR
# -----------------------------
def generate_task():
    node_list = list(G.nodes)

    pl = random.choice(node_list)
    dl = random.choice(node_list)

    while dl == pl:
        dl = random.choice(node_list)

    deadline = random.uniform(15, 60)

    return {"PL": pl, "DL": dl, "deadline": deadline}

# -----------------------------
# 5. COMPUTE TIMES (A*)
# -----------------------------
def compute_times(robot_name, task, robots):
    start = robots[robot_name]["start"]
    speed = robots[robot_name]["speed"]

    d1 = nx.astar_path_length(G, start, task["PL"], heuristic=heuristic, weight='weight')
    d2 = nx.astar_path_length(G, task["PL"], task["DL"], heuristic=heuristic, weight='weight')

    return d1/speed, d2/speed

# -----------------------------
# 6. MILP (SOFT DEADLINE)
# -----------------------------
def solve_milp(r1, r2, task_time, deadline):

    pt = [r1 + task_time, r2 + task_time]
    K = range(2)

    model = gp.Model()
    model.setParam("OutputFlag", 0)

    X = model.addVars(K, vtype=GRB.BINARY)
    Cmax = model.addVar()
    L = model.addVar()

    model.addConstr(sum(X[k] for k in K) == 1)
    model.addConstr(sum(X[k]*pt[k] for k in K) <= Cmax)
    model.addConstr(sum(X[k]*pt[k] for k in K) <= deadline + L)
    model.addConstr(L >= 0)

    model.setObjective(Cmax + 10*L, GRB.MINIMIZE)
    model.optimize()

    for k in K:
        if X[k].x > 0.5:
            return k, pt[k], Cmax.x, L.x

# -----------------------------
# 7. VISUALIZATION
# -----------------------------
import matplotlib.animation as animation

def animate(task, winner, path_r1, path_r2, path_task, robots):

    pos = nx.get_node_attributes(G, 'pos')

    fig, ax = plt.subplots(figsize=(7,7))

    # Draw base graph
    nx.draw(G, pos, ax=ax, with_labels=True, node_size=500, node_color='lightgray')

    # Draw task path (static)
    nx.draw_networkx_edges(G, pos,
        edgelist=list(zip(path_task, path_task[1:])),
        edge_color='blue', width=3, ax=ax)

    # Robot markers
    r1_dot, = ax.plot([], [], 'ro', markersize=10)
    r2_dot, = ax.plot([], [], 'go', markersize=10)

    # Labels
    ax.set_title(f"Winner: R{winner+1}")
    ax.axis('off')

    # Convert paths to coordinates
    r1_coords = [pos[n] for n in path_r1]
    r2_coords = [pos[n] for n in path_r2]

    max_len = max(len(r1_coords), len(r2_coords))

    def update(frame):
        if frame < len(r1_coords):
            x1, y1 = r1_coords[frame]
            r1_dot.set_data(x1, y1)

        if frame < len(r2_coords):
            x2, y2 = r2_coords[frame]
            r2_dot.set_data(x2, y2)

        return r1_dot, r2_dot

    ani = animation.FuncAnimation(
        fig, update,
        frames=max_len,
        interval=600,   # speed (ms)
        repeat=False
    )

    plt.show()
def visualize(task, winner, path_r1, path_r2, path_task, robots):

    pos = nx.get_node_attributes(G, 'pos')

    plt.figure(figsize=(7,7))
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='lightgray')

    # Paths
    nx.draw_networkx_edges(G, pos,
        edgelist=list(zip(path_r1, path_r1[1:])),
        edge_color='red', width=3)

    nx.draw_networkx_edges(G, pos,
        edgelist=list(zip(path_r2, path_r2[1:])),
        edge_color='green', width=3, style='dashed')

    nx.draw_networkx_edges(G, pos,
        edgelist=list(zip(path_task, path_task[1:])),
        edge_color='blue', width=3)

    # Nodes
    nx.draw_networkx_nodes(G, pos, nodelist=[robots["R1"]["start"]], node_color='red')
    nx.draw_networkx_nodes(G, pos, nodelist=[robots["R2"]["start"]], node_color='green')
    nx.draw_networkx_nodes(G, pos, nodelist=[task["PL"]], node_color='orange')
    nx.draw_networkx_nodes(G, pos, nodelist=[task["DL"]], node_color='blue')

    labels = {
        robots["R1"]["start"]: "R1",
        robots["R2"]["start"]: "R2",
        task["PL"]: "PL",
        task["DL"]: "DL"
    }
    nx.draw_networkx_labels(G, pos, labels=labels, font_color="white")

    plt.title(f"Winner: R{winner+1}")
    plt.axis('off')
    plt.show()

# -----------------------------
# 8. RUN 10 TASKS
# -----------------------------
for i in range(10):

    print(f"\n===== TASK {i+1} =====")

    robots = generate_robots()
    task = generate_task()

    r1, task_time = compute_times("R1", task, robots)
    r2, _ = compute_times("R2", task, robots)

    winner, pt, cmax, L = solve_milp(r1, r2, task_time, task["deadline"])

    print("Robots:", robots)
    print("Task:", task)
    print("R1 time:", round(r1,2), "R2 time:", round(r2,2))
    print("Winner: R", winner+1)
    print("Lateness:", round(L,2))

    path_r1 = nx.astar_path(G, robots["R1"]["start"], task["PL"], heuristic=heuristic, weight='weight')
    path_r2 = nx.astar_path(G, robots["R2"]["start"], task["PL"], heuristic=heuristic, weight='weight')
    path_task = nx.astar_path(G, task["PL"], task["DL"], heuristic=heuristic, weight='weight')

    visualize(task, winner, path_r1, path_r2, path_task, robots)