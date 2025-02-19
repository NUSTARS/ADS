import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Clean and categorize the static load data

project_root = Path(__file__).parent 
data_dir = project_root / "data/static"

csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

df_list = [pd.read_csv(os.path.join(data_dir, file)) for file in csv_files]

dataframes = []

for file in csv_files:
    parts = file.replace(".csv", "").split("_")
    velocity = int(parts[2].replace("ftps", ""))
    actuation = int(parts[3].replace("actuation", "").replace("act", "")) 
    is_image = False 
    if len(parts) == 5:
        is_image = parts[4].lower() == "image" 
    
    df = pd.read_csv(data_dir / file)

    # Save the first three cols
    columns_to_convert = df.columns.difference(["Type", "Units", "Time"])
    df[columns_to_convert] = df[columns_to_convert].apply(pd.to_numeric, errors='coerce')

    # Remove the units header from every df
    df = df[df["Units"] != "USC / SI"]

    df["Velocity (fps)"] = velocity
    df["Actuation State"] = actuation
    df["Is Image"] = is_image
    df["Yaw"] = round(pd.to_numeric(df["Yaw"], errors="coerce"),1)
    df["Key"] = list(zip(df["Velocity (fps)"], df["Actuation State"], df["Yaw"], df["Is Image"]))

    dataframes.append(df)

merged_data = pd.concat(dataframes, ignore_index=True)

output_file = project_root / "merged_data.csv"
# merged_data.to_csv(output_file, index=False)

loads_df = merged_data[merged_data["Is Image"] == False].copy()
image_df = merged_data[merged_data["Is Image"] == True].copy()

# Compute base vehicle drag using loops
drag_dict = {}
for velocity in loads_df["Velocity (fps)"].unique():
    for yaw in loads_df["Yaw"].unique():
        for actuation in loads_df["Actuation State"].unique():
            if actuation == 0:
                # Base drag calculation when actuation state is 0
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

                drag_dict[(velocity, yaw)] = {
                    "Image Drag": image_drag,
                    "Top Mount Drag": top_mount_drag,
                    "Bottom Mount Drag": bottom_mount_drag, 
                    "Base Model Drag": base_model_drag
                }
            else:
                continue


additional_cols = []
for _, row in loads_df.iterrows():
    velocity = row["Velocity (fps)"]
    yaw = row["Yaw"]
    key = (row["Velocity (fps)"], row["Yaw"])
    actuation = row["Actuation State"]
    drag_data = drag_dict[key]

    # Initialize drag values
    load_cell_drag = row["WAFBC Drag"]
    image_drag = drag_data["Image Drag"]
    top_mount_drag = drag_data["Top Mount Drag"]
    bottom_mount_drag = drag_data["Bottom Mount Drag"]
    base_model_drag = drag_data["Base Model Drag"]
    
    # Compute for Actuation State == 0 (base drag)
    if actuation == 0:
        ads_drag = 0
    else:
        ads_drag = load_cell_drag - bottom_mount_drag - base_model_drag

    # Add nondimensional Re and Cd
    total_vehicle_drag = load_cell_drag - bottom_mount_drag
    q = row["Dynamic Pressure"]
    Cd = total_vehicle_drag / (144 * q * 1/4 * (5.15/12)**2 * 3.1415)

    reynolds_number = row["Reynolds Number per ft"] * 5.15/12
    
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
        "Cd": Cd
    })

drag_df = pd.DataFrame(additional_cols)

# Merge the drag_df with the loads_df to include the new 'ADS Drag' column
cleaned_data = pd.concat([loads_df.reset_index(drop=True), drag_df], axis=1)

# Save cleaned data with the additional ADS Drag calculations
output_file = project_root / "static_clean.csv"
cleaned_data.to_csv(output_file, index=False)

# plot the drag values