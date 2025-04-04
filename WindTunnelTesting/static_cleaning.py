import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Read all static load files
project_root = Path(__file__).parent 
data_dir = project_root / "data/static"
csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]
df_list = [pd.read_csv(os.path.join(data_dir, file)) for file in csv_files]

dataframes = []

# Extract metadata from file name and add to combined data frame
for file in csv_files:
    parts = file.replace(".csv", "").split("_")
    velocity = int(parts[2].replace("ftps", ""))
    actuation = int(parts[3].replace("actuation", "").replace("act", "")) 
    is_image = False 
    if len(parts) == 5:
        is_image = parts[4].lower() == "image" 
    
    df = pd.read_csv(data_dir / file)

    # Keep the first three columns and convert the rest to numeric
    columns_to_convert = df.columns.difference(["Type", "Units", "Time"])
    df[columns_to_convert] = df[columns_to_convert].apply(pd.to_numeric, errors='coerce')

    # Remove the units header from every df
    df = df[df["Units"] != "USC / SI"]

    # Add metadata columns
    df["Velocity (fps)"] = velocity
    df["Actuation State"] = actuation
    df["Is Image"] = is_image
    df["Yaw"] = round(pd.to_numeric(df["Yaw"], errors="coerce"),1)
    df["Key"] = list(zip(df["Velocity (fps)"], df["Actuation State"], df["Yaw"], df["Is Image"]))

    dataframes.append(df)

# Combine all dataframes
merged_data = pd.concat(dataframes, ignore_index=True)

# Separate image and load cell data
loads_df = merged_data[merged_data["Is Image"] == False].copy()
image_df = merged_data[merged_data["Is Image"] == True].copy()

# Produce a dictionary with image mount drag values
drag_dict = {}
for velocity in loads_df["Velocity (fps)"].unique():
    for yaw in loads_df["Yaw"].unique():
        for actuation in loads_df["Actuation State"].unique():
            if actuation == 0:
                if velocity == 350 and yaw != 0:
                    # Skip yaw angles for 350 fps since we dont have data
                    continue

                loads_subset = loads_df[(loads_df["Velocity (fps)"] == velocity) & 
                                         (loads_df["Yaw"] == yaw) & 
                                         (loads_df["Actuation State"] == actuation)]
                image_subset = image_df[(image_df["Velocity (fps)"] == velocity) & 
                                        (image_df["Yaw"] == yaw) & 
                                        (image_df["Actuation State"] == actuation)]
                
                load_cell_drag = loads_subset["WAFBC Drag"].mean()
                image_drag = image_subset["WAFBC Drag"].mean() if not image_subset.empty else None
                top_mount_drag = (image_drag - load_cell_drag) if image_drag is not None else None
                bottom_mount_drag = top_mount_drag
                base_model_drag = load_cell_drag - bottom_mount_drag

                # Save data to the dictionary
                drag_dict[(velocity, yaw)] = {
                    "Image Drag": image_drag,
                    "Top Mount Drag": top_mount_drag,
                    "Bottom Mount Drag": bottom_mount_drag, 
                    "Base Model Drag": base_model_drag
                }
            else:
                continue

l_tip_to_mount = 41.50/12 # ft

actuation_state_kluge = {
    0: 0.00,
    10: 3.76,
    20: 7.21,
    30: 11.96,
    40: 17.76,
    50: 33.37,
    60: 38.12,
    70: 58.37,
    80: 72.79,
    90: 93.89,
    100: 100.00
}

additional_cols = []
for _, row in loads_df.iterrows():
    velocity = row["Velocity (fps)"]
    yaw = row["Yaw"]
    key = (row["Velocity (fps)"], row["Yaw"])
    actuation = row["Actuation State"]
    drag_data = drag_dict[key]

    # Extract drag values
    load_cell_drag = row["WAFBC Drag"]
    image_drag = drag_data["Image Drag"]
    top_mount_drag = drag_data["Top Mount Drag"]
    bottom_mount_drag = drag_data["Bottom Mount Drag"]
    base_model_drag = drag_data["Base Model Drag"]
    
    # Compute drag added by ADS
    if actuation == 0:
        ads_drag = 0
    else:
        ads_drag = load_cell_drag - bottom_mount_drag - base_model_drag

    # Add nondimensional Re and Cd
    total_vehicle_drag = load_cell_drag - bottom_mount_drag
    q = row["Dynamic Pressure"]
    Cd = total_vehicle_drag / (144 * q * 1/4 * (5.15/12)**2 * 3.1415)
    reynolds_number = row["Reynolds Number per ft"] * 5.15/12

    # Compute for Cp from Tip
    yaw_moment = row["WAFBC Yaw"] # ft-lbf
    side_force = -1 * row["WAFBC Side"] # lbf
    l_mount_to_cp = yaw_moment / side_force
    l_tip_to_cp = l_tip_to_mount + l_mount_to_cp

    # print(f"Velocity: {velocity}, Yaw: {yaw}, Actuation: {actuation}, Cp from Tip: {l_tip_to_cp}")

    real_actuation_state = actuation_state_kluge[actuation]
    
    # Append drag values to the list
    additional_cols.append({
        "Load Cell Drag": load_cell_drag,
        "Base Model Drag": base_model_drag,
        "Image Drag": image_drag,
        "Top Mount Drag": top_mount_drag,
        "Bottom Mount Drag": bottom_mount_drag,
        "ADS Drag": ads_drag,
        "Total Vehicle Drag": total_vehicle_drag,
        "Reynolds Number": reynolds_number,
        "Cd": Cd,
        "Cp from Tip": l_tip_to_cp,
        "Real Actuation State": real_actuation_state
    })

# Create a new dataframe with the additional columns
drag_df = pd.DataFrame(additional_cols)

# Merge the cleaned data with the loads df
cleaned_data = pd.concat([loads_df.reset_index(drop=True), drag_df], axis=1)

# Save cleaned data as a new csv
output_file = project_root / "static_clean.csv"
cleaned_data.to_csv(output_file, index=False)