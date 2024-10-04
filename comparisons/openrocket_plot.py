import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import os
import seaborn as sns

sns.set_style("whitegrid")

def clean_openrocket_data(df, alpha):
    assert 0 <= alpha <= 1, 'Alpha must be between 0 and 1'

    # Skip rows with event information based on the '#' identifier in the 'Time' column
    df = df[~df['# Time (s)'].astype(str).str.startswith('#')]

    # Convert the 'Time' column to integers
    df["time"] = df['# Time (s)'].astype(float)

    # Apply Exponential Weighted Moving Average (EWMA) smoothing
    df["smoothed_altitude"] = df['Altitude (ft)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_velocity"] = df['Vertical velocity (ft/s)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_acceleration"] = df['Vertical acceleration (ft/s²)'].ewm(alpha=alpha, min_periods=1).mean()

    return df

def fluid_properties(altitude):
    """Air density based on altitude (ft)."""
    if altitude < 36152:
        T = 59 - 0.00356 * altitude  # Temperature in Fahrenheit
        p = 2116 * ((T + 459.7) / 518.6)**5.256  # Pressure in lbf/ft^2
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    elif altitude < 82345:
        T = -70
        p = 473.1 * np.exp(1.73 - 0.000048 * altitude)
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    else:
        rho = -1

    a = (1.4 * 8.314 * (T + 459.7) / 0.02896)**0.5  # Speed of sound in ft/s

    nu = 1.568e-5 * (T + 459.7) / 518.67 * (518.67 + 110.4) / (T + 110.4)  # Kinematic viscosity in ft^2/s

    return T, p, rho, a, nu

project_root = Path(__file__).parent  # Gets the current directory where the script is located
file_path = project_root / "../openrocket_data/"
csv_list = [file for file in os.listdir(file_path) if file.endswith('.csv')]

def comparison_plot():
    plt.figure(figsize=(10, 6))
    for current_csv in csv_list:
        print(file_path / current_csv)
        df = pd.read_csv(file_path / current_csv, skiprows=5)
        df_cleaned = clean_openrocket_data(df, 1)
        max_height = max(df_cleaned['smoothed_altitude'])
        current_label = f'{current_csv} - {max_height:.1f} ft'
        plt.plot(df_cleaned['time'][df_cleaned['smoothed_velocity'] > 0], df_cleaned['smoothed_altitude'][df_cleaned['smoothed_velocity'] > 0], label=current_label)
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [ft]')
    plt.show()

def max_velocity_and_max_q():
    df = pd.read_csv(file_path / "Disturbance_Sims_Base.csv", skiprows=5)
    df_cleaned = clean_openrocket_data(df, 1)
    velocity = max(df_cleaned['smoothed_velocity'])
    return 1

comparison_plot()
max_velocity_and_max_q()