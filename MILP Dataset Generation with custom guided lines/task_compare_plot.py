import argparse
import json
import os
import random
import re
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


TASK_SIZES = [10, 20, 30, 40, 50, 60]
DEFAULT_SEEDS = range(1)

BASE_DIR = Path(__file__).resolve().parent
TASKS_DIR = BASE_DIR / "Tasks"
METRICS_DIR = BASE_DIR / "Metrics"

SCHEDULERS = {
    "milp": {
        "script": "rand_milp_simulation.py",
        "metric_suffix": "MILP_metrics.json",
        "label": "MILP",
        "marker": "o",
    },
    "fcfs": {
        "script": "rand_xgboost_fifo_simulation.py",
        "metric_suffix": "xgboost_fifo_metrics.json",
        "label": "FCFS",
        "marker": "s",
    },
    "ranker": {
        "script": "rand_xgboost_ranker_simulation.py",
        "metric_suffix": "xgboost_ranker_metrics.json",
        "label": "Proposed Ranker",
        "marker": "^",
    },
}

PICKUP_NODES = ["Gate_1", "Gate_2"]
DROPOFF_NODES = ["Gate_3", "Gate_4"]
STORAGE_NODES = [
    "Yellow_1", "Yellow_2", "Yellow_3", "Yellow_4", "Yellow_5",
    "Purple_1", "Purple_2", "Purple_3", "Purple_4", "Purple_5",
    "Green_1", "Green_2", "Green_3", "Green_4",
    "Green_5", "Green_6", "Green_7",
    "Orange_1", "Orange_2", "Orange_3",
    "Orange_4", "Orange_5", "Orange_6", "Orange_7",
]


def generate_scenario(num_tasks, random_seed):
    """Create the scenario file used by all schedulers for this task size."""
    rng = random.Random(random_seed)
    tasks = []

    for task_id in range(1, num_tasks + 1):
        task_type = rng.choices([1, 2, 3], weights=[0.4, 0.4, 0.2])[0]

        if task_type == 1:
            pickup_location = rng.choice(PICKUP_NODES)
            dropoff_location = rng.choice(STORAGE_NODES)
        elif task_type == 2:
            pickup_location = rng.choice(STORAGE_NODES)
            dropoff_location = rng.choice(DROPOFF_NODES)
        else:
            pickup_location = rng.choice(STORAGE_NODES)
            dropoff_location = rng.choice(STORAGE_NODES)
            while dropoff_location == pickup_location:
                dropoff_location = rng.choice(STORAGE_NODES)

        tasks.append({
            "id": task_id,
            "type": task_type,
            "PL": pickup_location,
            "DL": dropoff_location,
            "pickup_time": 3,
            "dropoff_time": 3,
            "deadline": 300,
        })

    TASKS_DIR.mkdir(exist_ok=True)
    scenario_path = TASKS_DIR / f"new_{num_tasks}.json"
    with scenario_path.open("w") as f:
        json.dump(tasks, f, indent=4)

    return scenario_path


def metric_path(scheduler_name, num_tasks, random_seed):
    suffix = SCHEDULERS[scheduler_name]["metric_suffix"]
    return METRICS_DIR / f"new_{num_tasks}_random_seed_{random_seed}_{suffix}"


def run_scheduler(scheduler_name, num_tasks, random_seed):
    scheduler = SCHEDULERS[scheduler_name]
    script_path = BASE_DIR / scheduler["script"]
    output_path = metric_path(scheduler_name, num_tasks, random_seed)

    if output_path.exists():
        output_path.unlink()

    source = script_path.read_text()
    source = re.sub(
        r'^file_name\s*=\s*["\'][^"\']+["\']',
        f'file_name = "new_{num_tasks}"',
        source,
        count=1,
        flags=re.MULTILINE,
    )
    source = re.sub(
        r"^random_seed\s*=\s*\d+",
        f"random_seed = {random_seed}",
        source,
        count=1,
        flags=re.MULTILINE,
    )

    runner = (
        "import matplotlib\n"
        "matplotlib.use('Agg')\n"
        "import matplotlib.pyplot as plt\n"
        "plt.show = lambda *args, **kwargs: None\n"
        "from pathlib import Path\n"
        f"code = {source!r}\n"
        f"exec(compile(code, {str(script_path)!r}, 'exec'))\n"
        f"metric_path = Path({str(output_path)!r})\n"
        "for frame in range(1, 200000):\n"
        "    update(frame)\n"
        "    if metric_path.exists():\n"
        "        break\n"
        "plt.close('all')\n"
    )

    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"

    completed = subprocess.run(
        [sys.executable, "-c", runner],
        cwd=BASE_DIR,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    if completed.returncode != 0:
        raise RuntimeError(
            f"{scheduler['label']} failed for {num_tasks} tasks, "
            f"seed {random_seed}.\n\nSTDOUT:\n{completed.stdout}\n\n"
            f"STDERR:\n{completed.stderr}"
        )

    if not output_path.exists():
        raise FileNotFoundError(
            f"{scheduler['label']} finished but did not create {output_path}"
        )

    with output_path.open() as f:
        metrics = json.load(f)

    return metrics


def collect_results(task_sizes, seeds):
    results = []
    METRICS_DIR.mkdir(exist_ok=True)

    for num_tasks in task_sizes:
        times = {scheduler_name: [] for scheduler_name in SCHEDULERS}

        for random_seed in seeds:
            scenario_path = generate_scenario(num_tasks, random_seed)
            print(f"Scenario: {scenario_path.relative_to(BASE_DIR)}")

            for scheduler_name in SCHEDULERS:
                metrics = run_scheduler(
                    scheduler_name,
                    num_tasks,
                    random_seed,
                )
                computation_time = metrics["computation_time"]
                times[scheduler_name].append(computation_time)
                print(
                    f"  {SCHEDULERS[scheduler_name]['label']}: "
                    f"{computation_time:.6f} s"
                )

        row = {"tasks": num_tasks}
        for scheduler_name, scheduler_times in times.items():
            row[scheduler_name] = np.mean(scheduler_times)
            row[f"{scheduler_name}_std"] = np.std(scheduler_times)

        results.append(row)

    return pd.DataFrame(results)


def plot_results(df, output_path, use_log_scale=True):
    plt.figure(figsize=(8, 5))

    for scheduler_name, scheduler in SCHEDULERS.items():
        plt.errorbar(
            df["tasks"],
            df[scheduler_name],
            yerr=df[f"{scheduler_name}_std"],
            marker=scheduler["marker"],
            linewidth=2,
            capsize=4,
            label=scheduler["label"],
        )

    plt.xlabel("Number of Tasks")
    plt.ylabel("Computation Time (s)")
    plt.title("Scalability Comparison")

    if use_log_scale:
        plt.yscale("log")

    plt.grid(alpha=0.3, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate task scenarios, run MILP/FCFS/Ranker on the same "
            "scenario, and plot average computation time with error bars."
        )
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=len(DEFAULT_SEEDS),
        help="Number of random seeds to run for each task size.",
    )
    parser.add_argument(
        "--linear",
        action="store_true",
        help="Use a linear y-axis instead of the default logarithmic scale.",
    )
    parser.add_argument(
        "--output",
        default="task_scalability_computation_time.png",
        help="Path for the saved plot.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    seeds = range(args.repeats)

    df = collect_results(TASK_SIZES, seeds)
    print()
    print(df)

    csv_path = BASE_DIR / "task_scalability_computation_time.csv"
    df.to_csv(csv_path, index=False)

    output_path = BASE_DIR / args.output
    plot_results(
        df,
        output_path,
        use_log_scale=not args.linear,
    )

    print()
    print(f"Results saved to {csv_path}")
    print(f"Plot saved to {output_path}")


if __name__ == "__main__":
    main()
