import os
from pathlib import Path
import pandas as pd

project_root = Path(__file__).parent 
data_dir = project_root / "data"

# List all CSV files in the directory
csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

# Read all CSVs and concatenate them
df_list = [pd.read_csv(os.path.join(data_dir, file)) for file in csv_files]

dataframes = []

for file in csv_files:
    # print(f"Reading {file}")
    parts = file.replace(".csv", "").split("_")
    # print(parts)
    velocity = int(parts[2].replace("ftps", ""))
    actuation = int(parts[3].replace("actuation", "").replace("act", "")) 
    is_image = False 
    if len(parts) == 5:
        is_image = parts[4].lower() == "image" 
    
    df = pd.read_csv(data_dir / file)
    df["Velocity (fps)"] = velocity
    df["Actuation State"] = actuation
    df["Is Image"] = is_image

    # print(f"Read {file} with {len(df)} rows")
    # print(f"Velocity: {velocity}, Actuation: {actuation}, Image: {is_image}")

    dataframes.append(df)

final_df = pd.concat(dataframes, ignore_index=True)

# create a dictionary for the lookup drag values

# for a given velocity
# find the 0 actuation state
# select the image run and the non image run
# take the 0 yaw value

velocity_range = [50, 100, 150, 200, 250, 300]
actuation_range = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

for current_velocity in velocity_range:
    for current_actuation in actuation_range:
        # find the 0 actuation state
        loads_df = final_df[(final_df["Velocity (fps)"] == current_velocity) & (final_df["Actuation State"] == current_actuation)]
        loads_df = loads_df[loads_df["Is Image"] == False]
        loads_df = loads_df[loads_df["Yaw"] == 0]

        # select the image run
        image_df = final_df[(final_df["Velocity (fps)"] == current_velocity) & (final_df["Actuation State"] == current_actuation)]
        image_df = image_df[image_df["Is Image"] == True]
        image_df = image_df[image_df["Yaw"] == 0]

        print(f"Velocity: {current_velocity}, Actuation: {current_actuation}")
        print(f"Non Image Run: {df['Drag'].mean()}")
        print(f"Image Run: {image_df['Drag'].mean()}")

        print(loads_df.head(10))
        print(image_df.head(10))
        break