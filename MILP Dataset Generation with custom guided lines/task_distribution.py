import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ===========================
# Load Dataset
# ===========================
df = pd.read_csv("All Datasets/working_dataset_tr_kkkw.csv")

# ===========================
# Count Group Sizes
# ===========================
group_counts = (
    df["group_size"]
    .value_counts()
    .sort_index()
    .reindex(range(1, 9), fill_value=0)
)

total = group_counts.sum()

# ===========================
# Colors
# ===========================
colors = plt.cm.Blues(np.linspace(0.35, 0.90, len(group_counts)))

# ===========================
# Portrait Figure
# ===========================
fig, ax = plt.subplots(
    figsize=(8, 9),      # Portrait canvas
    dpi=300
)

# Make the plotting area portrait
ax.set_box_aspect(1)

# ===========================
# Bars
# ===========================
bars = ax.bar(
    group_counts.index,
    group_counts.values,
    width=0.70,
    color=colors,
    edgecolor="#444444",
    linewidth=0.8
)

# ===========================
# Title & Labels
# ===========================
ax.set_title(
    "Distribution of Group Sizes",
    fontsize=18,
    fontweight="bold",
    pad=20
)

ax.set_xlabel(
    "Group Size",
    fontsize=14,
    labelpad=10
)

ax.set_ylabel(
    "Number of Tasks",
    fontsize=14,
    labelpad=10
)

ax.set_xticks(range(1, 9))
ax.tick_params(axis='both', labelsize=12)

# ===========================
# Grid
# ===========================
ax.grid(
    axis="y",
    linestyle="--",
    linewidth=0.8,
    alpha=0.25
)

ax.set_axisbelow(True)

# ===========================
# Limits
# ===========================
ax.set_ylim(
    0,
    group_counts.max() * 1.18
)

# ===========================
# Value Labels
# ===========================
for bar in bars:

    height = bar.get_height()
    percentage = height / total * 100

    ax.text(
        bar.get_x() + bar.get_width()/2,
        height + group_counts.max()*0.015,
        f"{height:,}\n({percentage:.1f}%)",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#333333"
    )

# ===========================
# Style
# ===========================
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

ax.spines["left"].set_color("#BBBBBB")
ax.spines["bottom"].set_color("#BBBBBB")

plt.tight_layout()

# ===========================
# Save
# ===========================
plt.savefig(
    "group_size_distribution_portrait.png",
    dpi=600
)

plt.show()