import json
import random

TOTAL_TASKS = 15

PICKUP_NODES = [
    "Gate_1",
    "Gate_2",
]

DROPOFF_NODES = [
    "Gate_3",
    "Gate_4",
]

STORAGE_NODES = [
    "Yellow_1", "Yellow_2", "Yellow_3", "Yellow_4", "Yellow_5",
    "Purple_1", "Purple_2", "Purple_3", "Purple_4", "Purple_5",
    "Green_1", "Green_2", "Green_3", "Green_4",
    "Green_5", "Green_6", "Green_7",
    "Orange_1", "Orange_2", "Orange_3",
    "Orange_4", "Orange_5", "Orange_6", "Orange_7",
]

random.seed(42)

tasks = []

for task_id in range(1, TOTAL_TASKS + 1):

    task_type = random.choices(
        [1, 2, 3],
        weights=[0.4, 0.4, 0.2]
    )[0]

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

    tasks.append({

        "id": task_id,

        "type": task_type,

        "PL": pl,

        "DL": dl,

        "pickup_time": 3,

        "dropoff_time": 3,

        "deadline": 300

    })

with open(f"Tasks/new_{TOTAL_TASKS}.json", "w") as f:

    json.dump(
        tasks,
        f,
        indent=4
    )

print("Scenario saved.")