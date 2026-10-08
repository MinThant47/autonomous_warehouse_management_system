import json
from scipy.stats import spearmanr

# ============================================
# Load schedules
# ============================================

with open("milp_schedule.json", "r") as f:
    milp = json.load(f)

with open("xgboost_classifier_ranker_schedule.json", "r") as f:
    ranker = json.load(f)


# ============================================
# Compare one robot
# ============================================

def compare_robot(robot):

    print("\n" + "=" * 60)
    print(robot)
    print("=" * 60)

    milp_tasks = milp[robot]
    ranker_tasks = ranker[robot]

    milp_rank = {
        t["task_id"]: i + 1
        for i, t in enumerate(milp_tasks)
    }

    ranker_rank = {
        t["task_id"]: i + 1
        for i, t in enumerate(ranker_tasks)
    }

    common = sorted(set(milp_rank.keys()) & set(ranker_rank.keys()))

    exact = 0
    abs_error = 0

    milp_order = []
    ranker_order = []

    print(
        f"{'Task':<8}"
        f"{'MILP':<8}"
        f"{'Ranker':<10}"
        f"{'Difference':<10}"
    )

    print("-" * 45)

    for task in common:

        m = milp_rank[task]
        r = ranker_rank[task]

        diff = abs(m-r)

        milp_order.append(m)
        ranker_order.append(r)

        abs_error += diff

        if m == r:
            exact += 1

        print(
            f"{task:<8}"
            f"{m:<8}"
            f"{r:<10}"
            f"{diff:<10}"
        )

    # ----------------------------------------

    rho, _ = spearmanr(
        milp_order,
        ranker_order
    )

    print("\nResults")
    print("------------------------")

    print(
        f"Tasks                 : {len(common)}"
    )

    print(
        f"Exact position        : "
        f"{exact}/{len(common)} "
        f"({100*exact/len(common):.2f}%)"
    )

    print(
        f"Average rank error    : "
        f"{abs_error/len(common):.2f}"
    )

    print(
        f"Spearman correlation  : "
        f"{rho:.3f}"
    )

    # ----------------------------------------
    # Top-3 overlap
    # ----------------------------------------

    top3_milp = set(
        task
        for task, rank in milp_rank.items()
        if rank <= 3
    )

    top3_ranker = set(
        task
        for task, rank in ranker_rank.items()
        if rank <= 3
    )

    overlap = len(
        top3_milp & top3_ranker
    )

    print(
        f"Top-3 overlap         : "
        f"{overlap}/3"
    )


# ============================================

compare_robot("R1")

compare_robot("R2")