import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
'''
Plotting function to plot data based on the filtered dataframe
'''
def plot_data(filtered_df, x_col, y_col, group_by, scatter, title, x_axis, y_axis):
    groups = filtered_df.groupby(group_by)
    plt.figure(figsize=(10, 6))
    for group_by, group in groups:
        sorted_group = group.sort_values(by=x_col)
        if scatter:
            plt.scatter(sorted_group[x_col], sorted_group[y_col]*12, label=f"{group_by}", marker='o')
        else:
            plt.plot(sorted_group[x_col], sorted_group[y_col]*12, label=f"{group_by}", marker='o')
    plt.xlabel(x_axis)
    plt.ylabel(y_axis)
    plt.title(title)
    plt.grid(True)
    plt.legend(loc='best', fontsize='small', frameon=True)

project_root = Path(__file__).parent 
file_path = project_root / "static_clean.csv"
df = pd.read_csv(file_path)

filtered_df = df[(df["Yaw"].isin([6])) & (df["Actuation State"].isin([0, 50, 100]))]
plot_data(filtered_df, "Reynolds Number", "Cp from Tip", "Actuation State", False, "Cp Distance vs. Reynolds Number for Yaw=6 and Selected Actuation States", "Reynolds Number", "CP Distance [in]")
plt.savefig("CpDistance_vs_ReynoldsNumber.png", dpi=300)

plt.show()