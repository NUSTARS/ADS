import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Read in all static load files
project_root = Path(__file__).parent 
df = pd.read_csv("WindTunnelTesting/data/misc/021725_100423_350ftps_sweep.csv", header=None)
# df = pd.read_csv("WindTunnelTesting/data/misc/021725_100423_350ftps_sweep.csv", skiprows=2, header=None)

df_header = df.iloc[:2].copy()
df_data = df.iloc[2:].copy()

actuation_states = [100, 88, 80, 70, 60, 50, 40, 30, 20, 10, 0, 0] 

for i, (_, row) in enumerate(df_data.iterrows()):
    expected_actuation_state = actuation_states[i]  # Safer than pop()

    # Ensure row is treated as a DataFrame before concatenation
    df_current = pd.concat([df_header, row.to_frame().T], axis=0)

    # Use the first row as columns, then drop it
    df_current.columns = df_current.iloc[0]
    df_current = df_current[1:]

    print(df_current.head())  # Debugging step

    file_name = f"WindTunnelTesting/data/static/021725_manufactured_350ftps_{expected_actuation_state}actuation.csv"
    df_current.to_csv(file_name, index=False)  # Save to CSV
