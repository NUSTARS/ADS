import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

project_root = Path(__file__).parent 

file_path = project_root / "final_static_data.csv"
df = pd.read_csv(file_path)

# Filter data for Yaw = 0 and Actuation States 0, 50, and 100
filtered_df = df[(df["Yaw"] == 0) & (df["Actuation State"].isin([0, 20, 50, 70, 100]))]

# Group data by Actuation State
groups = filtered_df.groupby("Actuation State")

L = 5.15/12
A = 1/4 * L**2 * 3.1415

# Plot
plt.figure(figsize=(10, 6))
for actuation, group in groups:
    sorted_group = group.sort_values(by="Reynolds Number per ft")
    plt.plot(sorted_group["Reynolds Number per ft"]*L, sorted_group["ADS Drag"], label=f"Actuation {actuation}", marker='o')

plt.xlabel("Reynolds Number")
plt.ylabel("Drag Force (ADS)")
plt.title("Drag Force vs. Reynolds Number for Yaw=0 and Selected Actuation States")
plt.legend(loc='best', fontsize='small', frameon=True)
plt.grid(True)

plt.figure(figsize=(10, 6))
for actuation, group in groups:
    sorted_group = group.sort_values(by="Reynolds Number per ft")
    F = sorted_group["Total Vehicle Drag"]
    q = sorted_group["Dynamic Pressure"]
    Cd = F / (144 * q * A) 
    plt.plot(sorted_group["Reynolds Number per ft"]*L, Cd, label=f"Actuation {actuation}", marker='o')

plt.xlabel("Reynolds Number")
plt.ylabel("Cd")
plt.title("Cd vs. Reynolds Number for Yaw=0 and Selected Actuation States")
plt.legend(loc='best', fontsize='small', frameon=True)
plt.grid(True)
plt.show()

