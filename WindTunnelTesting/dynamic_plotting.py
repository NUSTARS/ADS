import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

project_root = Path(__file__).parent 
data_dir = project_root / "data/dynamic"

csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

df = pd.read_csv(data_dir / csv_files[0], skiprows=2)

if "Time" not in df.columns:  # Handle missing headers
    df = pd.read_csv(data_dir / csv_files[0], skiprows=1, names=["Time", "WAFBC Drag"])

df["Time"] = df["Time"].str.strip()  # Remove unwanted spaces

# Convert "Time" column (format: YYMMDD HH:MM:SS.sss) to datetime
df["Datetime"] = pd.to_datetime(df["Time"], format="%y%m%d %H:%M:%S.%f", errors="coerce")

# Ensure no NaT values (debugging)
if df["Datetime"].isna().sum() > 0:
    print("Warning: Some timestamps failed to parse.")
    print(df[df["Datetime"].isna()])

# Convert to elapsed time (seconds) from the first timestamp
df["Elapsed Time (s)"] = (df["Datetime"] - df["Datetime"].iloc[0]).dt.total_seconds()

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(df["Elapsed Time (s)"], df["WAFBC Drag"], marker='o', label="WAFBC Drag")
plt.xlabel("Time [s]")
plt.ylabel("Drag [lbf]")
plt.title("Drag vs Time History")
plt.legend(loc='best', fontsize='small', frameon=True)
plt.grid(True)
plt.show()