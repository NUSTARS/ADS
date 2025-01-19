import subprocess
import csv
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import os

sim_results = []
OR_results = []

csv_file_path = "IC_data.csv"  # Input CSV file
constants_file_path = "constants.h"  # Constants header file
cpp_executable = "./build/my_program"  # Path to compiled C++ executable
build_directory = "./build"

patterns = {
    "BALLAST_MASS": r"const double BALLAST_MASS = .*?;",
    "WIND_VELOCITY": r"const double WIND_VELOCITY = .*?;",
    "WIND_ANGLE": r"const double WIND_ANGLE = .*?;",
    "WIND_STD": r"const double WIND_STD = .*?;"
}

with open(csv_file_path, 'r') as csv_file:
    reader = csv.DictReader(csv_file)  # Use DictReader for named access
    header = reader.fieldnames

    for row in reader:
        new_values = {
            "BALLAST_MASS": float(row["BALLAST_MASS"])/32.17,
            "WIND_VELOCITY": float(row["WIND_VELOCITY"]),
            "WIND_ANGLE": float(row["WIND_ANGLE"]),
            "WIND_STD": float(row["WIND_STD"])
        }

        with open(constants_file_path, 'r') as file:
            lines = file.readlines()

        updated_lines = []
        for line in lines:
            for key, pattern in patterns.items():
                if re.search(pattern, line):
                    # Replace the constant's value
                    line = re.sub(pattern, f"const double {key} = {new_values[key]};", line)
            updated_lines.append(line)

        # Write the updated content back to the file
        with open(constants_file_path, 'w') as file:
            file.writelines(updated_lines)

        # print("Updated constants.h with values:", new_values)

        # print("Compiling the C++ program with make...")
        try:
            compilation_result = subprocess.run(
                ["make"],
                cwd=build_directory,  # Change directory to the build folder
                capture_output=True,
                text=True
            )
            if compilation_result.returncode != 0:
                print("Compilation failed!")
                print(compilation_result.stderr)
                continue
        except FileNotFoundError:
            print("Error: `make` command not found or build directory does not exist.")
            break


        # Extract other simulation inputs from the row
        inputs = [
            row["Lateral velocity (ft/s)"],
            row["Vertical velocity (ft/s)"],
            row["Pitch rate (r/s)"],
            row["Yaw rate (r/s)"],
            row["Lateral orientation (azimuth) (°)"],
            row["Vertical orientation (zenith) (°)"],
            row["Altitude (ft)"]
        ]

        # Run the C++ program with the inputs
        result = subprocess.run(
            [cpp_executable] + inputs,
            capture_output=True,
            text=True
        )

        OR_altitude = row["OR Altitude"]

        # Print the output from the C++ program
        altitude = result.stdout
        sim_results.append(altitude)
        OR_results.append(OR_altitude)
        print("Simulation output:", altitude)
        print("OR output:", OR_altitude)
        print(f"Difference: {round(float(altitude) - float(OR_altitude), 2)} ft")

difference = np.array(sim_results, dtype=float) - np.array(OR_results, dtype=float)
plt.figure(figsize=(6, 4))
plt.boxplot(difference, vert=False, patch_artist=True, boxprops=dict(facecolor='skyblue'))
plt.scatter(difference, [1] * len(difference), color='black', alpha=0.6)
plt.xlabel('Altitude Difference (ft)')
plt.show()