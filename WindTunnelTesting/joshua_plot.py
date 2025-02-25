import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import seaborn as sns

project_root = Path(__file__).parent 
file_path = project_root / "static_clean.csv"
df = pd.read_csv(file_path)

filtered_df = df[(df["Yaw"] == 0) & (df["Actuation State"].isin([0, 50, 100]))]
palette = sns.color_palette("tab10")

# groups = filtered_df.groupby("Actuation State")
groups = dict(sorted(filtered_df.groupby("Actuation State"), key=lambda x: x[0], reverse=True))

plt.figure(figsize=(4, 5))

# Increase text size
plt.rcParams.update({'font.size': 14})

for i, (group_by, group) in enumerate(groups.items()):
    sorted_group = group.sort_values(by="Reynolds Number")
    plt.plot(
        sorted_group['Reynolds Number'], 
        sorted_group["Total Vehicle Drag"], 
        label=f"ADS {group_by}%", 
        marker='o', 
        color=palette[i % len(palette)]  # Cycle through colors
    )

plt.xlabel("Reynolds Number", fontsize=16)
plt.ylabel("Drag Force [lbf]", fontsize=16)
plt.title("Effect of ADS", fontsize=18)
plt.grid(True)
plt.legend(loc='best', fontsize=12, frameon=True)
plt.ticklabel_format(axis='x', style='sci', scilimits=(0, 0))
plt.tight_layout()
plt.savefig("static_ads.png", dpi=300)

plt.show()