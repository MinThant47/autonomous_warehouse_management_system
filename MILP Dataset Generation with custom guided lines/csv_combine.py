import pandas as pd
import glob
import os

# Folder containing the 18 csv files
folder = "/Users/minthantkyaw/Documents/Operation Files/YTU/2025-26 Final Year/Python/MILP Dataset Generation with custom guided lines/working/Working Dataset"

csv_files = sorted(glob.glob(os.path.join(folder, "*.csv")))

dfs = []

print("=" * 60)
print("Processing CSV files")
print("=" * 60)

for run_id, file in enumerate(csv_files):

    df = pd.read_csv(file)

    # Make every group_id unique by prefixing the run number
    df["group_id"] = (
        f"RUN{run_id:02d}_"
        + df["group_id"].astype(str)
    )

    dfs.append(df)

    print(
        f"RUN{run_id:02d} | "
        f"{os.path.basename(file)} | "
        f"Rows = {len(df)} | "
        f"Groups = {df['group_id'].nunique()}"
    )

print("\nConcatenating...")

combined = pd.concat(
    dfs,
    ignore_index=True
)

print("=" * 60)
print("FINAL DATASET")
print("=" * 60)

print("Rows:", len(combined))
print("Unique group ids:", combined["group_id"].nunique())

group_sizes = combined.groupby("group_id").size()

print("\nActual group sizes")
print(group_sizes.describe())

print("\nDistribution")
print(group_sizes.value_counts().sort_index())

combined.to_csv(
    "/Users/minthantkyaw/Documents/Operation Files/YTU/2025-26 Final Year/Python/MILP Dataset Generation with custom guided lines/working/real_balanced_dataset.csv",
    index=False
)

print("\n✅ Saved as:")
print("unique_combined_dataset.csv")