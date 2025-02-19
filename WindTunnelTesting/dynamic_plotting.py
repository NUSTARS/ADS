import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

project_root = Path(__file__).parent 
data_dir = project_root / "data/dynamic"

csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

df = pd.read_csv(data_dir / csv_files[0])

df["WAFBC Drag"] = pd.to_numeric(df["WAFBC Drag"], errors='coerce')

print(df.head())

# Plot the data
plt.figure(figsize=(10, 6))
plt.scatter(df["Seconds"], df["WAFBC Drag"], marker='o', label="WAFBC Drag")
plt.xlabel("Time [s]")
plt.ylabel("Drag [lbf]")
plt.title("Drag vs Time History")
plt.legend(loc='best', fontsize='small', frameon=True)
plt.grid(True)
plt.show()

# waiting to downsample data