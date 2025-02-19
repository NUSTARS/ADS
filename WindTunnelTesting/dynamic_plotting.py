import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

project_root = Path(__file__).parent 
data_dir = project_root / "data/dynamic"

csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

for csv in csv_files:

    df = pd.read_csv(data_dir / csv)

    df["WAFBC Drag"] = pd.to_numeric(df["WAFBC Drag"], errors='coerce')

    Q1 = df["WAFBC Drag"].quantile(0.25)
    Q3 = df["WAFBC Drag"].quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    df = df[(df["WAFBC Drag"] >= lower_bound) & (df["WAFBC Drag"] <= upper_bound)]

    # Apply a rolling average to smooth data (window size = 10)
    df["Smoothed Drag"] = df["WAFBC Drag"].rolling(window=10, min_periods=1).mean()

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.scatter(df["Seconds"], df["WAFBC Drag"], marker='o', alpha=0.3, label="Raw Data")
    plt.plot(df["Seconds"], df["Smoothed Drag"], color='red', linewidth=2, label="Smoothed Data")
    plt.xlabel("Time [s]")
    plt.ylabel("Drag [lbf]")
    plt.title("Drag vs Time History (Smoothed)")
    plt.legend(loc='best', fontsize='small', frameon=True)
    plt.grid(True)

plt.show()