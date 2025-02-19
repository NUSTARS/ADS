import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Step 1: Load the CSV file
# Replace 'your_file.csv' with the path to your CSV file
file_path = 'data.csv'

names = ["time", "altitude", "vx", "vy", "vz", "wx", "wy", "wz", "thetax", "thetay", "thetaz", "windx", "windy", "u"]

data = pd.DataFrame(columns=names)

file_path_OR = 'OR_data.csv' 
data_OR = pd.read_csv(file_path_OR)


# Read data from CSV
csv_data = pd.read_csv(file_path)

if list(csv_data.columns) != names:
    csv_data.columns = names  # Assign names if CSV does not have headers or has mismatched headers

data = pd.concat([data, csv_data], ignore_index=True)
print(data)

# Step 2: Extract the x-axis data
# Replace 'X' with the actual column name for the x-axis
x = data["time"]  # X-axis data

fig, axes = plt.subplots(2, 2, figsize=(6,5))  # Create 2x2 subplots
fig.suptitle('Whole Lotta Data', fontsize=16)    # Add a main title

# Plot on each subplot
# Top-left
# axes[0, 0].plot(x, data['altitude'], label='6DOF Simulation', color='b')
# #axes[0, 0].plot(data_OR['# Time (s)']-2.309, data_OR['Altitude (ft)'], label='OR', color='g')
# axes[0, 0].set_title('Altitude')
# axes[0, 0].set_xlabel('time (s)')
# axes[0, 0].set_ylabel('alt (ft)')

# print(x)
# print(data['windx'])

axes[0, 0].plot(x, data['windx'], color='b',label='windx')
axes[0, 0].plot(x, data['windy'], color='g',label='windy')
axes[0, 0].legend()
axes[0, 0].grid(True)

# axes[0, 0].plot(x, data['u'], color='b',label='u')
# axes[0, 0].legend()
# axes[0, 0].grid(True)


# Top-right
axes[0, 1].plot(x, data['vx'], color='b',label='vx')
axes[0, 1].plot(x, data['vy'], color='g',label='vy')
axes[0, 1].plot(x, data['vz'], color='r',label='vz')
# axes[0,1].plot(data_OR['# Time (s)']-2.309, data_OR['Vertical velocity (ft/s)'], label='vertical velocity', linestyle='--', color='r')
axes[0, 1].set_title('Velocities')
axes[0, 1].set_xlabel('time')
axes[0, 1].set_ylabel('vel')
axes[0, 1].legend()
axes[0, 1].grid(True)

# Bottom-left
axes[1,0].plot(x, data['wx'], color='b',label='wx')
axes[1,0].plot(x, data['wy'], color='g',label='wy')
axes[1,0].plot(x, data['wz'], color='r',label='wz')
axes[1,0].set_title('Omegas')
axes[1,0].set_xlabel('time')
axes[1,0].set_ylabel('omega')
axes[1,0].legend()
axes[1,0].grid(True)

# Bottom-right
axes[1,1].plot(x, data['thetax'], color='b',label='thetax')
axes[1,1].plot(x, data['thetay'], color='g',label='thetay')
axes[1,1].plot(x, data['thetaz'], color='r',label='thetaz')
axes[1,1].set_title('Theta')
axes[1,1].set_xlabel('time')
axes[1,1].set_ylabel('theta')
axes[1,1].legend()
axes[1,1].grid(True)

# Adjust layout to prevent overlap
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout and leave space for main title

plt.show()

# Step 4: Show or save the figure
for i in plt.get_fignums():
    plt.figure(i).savefig(f'Output_Figure_{i}.png', dpi=300)

