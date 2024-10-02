import matplotlib.pyplot as plt
import pandas as pd

def compute_apogee(time_in, height_in, velocity_in, acceleration_in, mass, area, dt=0.01):
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = 0.002378  # density of air in slugs/ft^3
    Cd = 0.3  # drag coefficient
    
    time_step = dt  # time step for Euler integration
    
    # Initialize variables
    current_velocity = velocity_in
    current_height = height_in
    current_time = time_in
    
    times = []
    heights = []
    velocities = []
    
    while current_velocity > 0:
        times.append(current_time)
        F_drag = 0.5 * rho_air * current_velocity**2 * Cd * area
        F_gravity = mass * g
        F_net = -1 * F_drag - F_gravity
        acceleration_net = F_net / mass
        current_velocity = current_velocity + acceleration_net * time_step
        current_height = current_height + current_velocity * time_step
        current_time += time_step
        
        # Save current height and velocity
        heights.append(current_height)
        velocities.append(current_velocity)

    return times, heights, velocities

file_name = "FT3_primary.csv"
df = pd.read_csv(file_name)
df['altitude_ft'] = df['altitude'] * 3.28084
df['height_ft'] = df['height'] * 3.28084
df['velocity_ft/s'] = df['speed'] * 3.28084
df['acceleration_ft/s^2'] = df['acceleration'] * 3.28084 
df_isolated = df[(df['state_name'].str.strip() == 'fast') | (df['state_name'].str.strip() == 'coast')]
# print(df_isolated.head())

mass = 37.15/32.17  # Mass of the rocket in slugs (1 slug = 32.174 lbs·s^2/ft)
area = 0.25  # Cross-sectional area of the rocket in ft^2

predicted_trajectories = []

for index, row in df_isolated.iterrows():
    current_time = row['time']
    current_height = row['height_ft']  # Initial height in feet
    current_velocity = row['velocity_ft/s']  # Initial velocity in ft/s
    current_acceleration = row['acceleration_ft/s^2']  # Acceleration in ft/s^2

    # Compute trajectory for this time step
    times, heights, velocities = compute_apogee(current_time, current_height, current_velocity, current_acceleration, mass, area, 0.01)

    label_time = current_time
    
    # Append predicted trajectory data to the list
    # predicted_trajectories.append((times, heights, f"Predicted Apogee: {max(heights):.2f} feet"))
    predicted_trajectories.append((times, heights, label_time))

# Plot all trajectories on the same figure
plt.figure()

# Plot predicted trajectories
counter = 0
for times, heights, label_time in predicted_trajectories:
    if counter % 150 == 0:
        plt.plot(times, heights, label=f'Time: {label_time:.2f} seconds')
    counter += 1

# Plot the entire trajectory from the DataFrame
plt.plot(df_isolated['time'], df_isolated['height_ft'], label="Actual Trajectory", color='red', linewidth=2)

plt.xlabel("Time (seconds)")
plt.ylabel("Height (feet)")
plt.title("Rocket Trajectories during Coast Phase")
plt.legend()
plt.grid(True)

print("df_isolated['height_ft'] contents: ", df_isolated["height_ft"].tolist())

# Calculate the maximum height from the DataFrame
max_height = max(df_isolated["height_ft"].tolist())

# Initialize lists to store time and relative error
error_times = []
relative_errors = []

# Calculate relative error for each predicted trajectory
for times, heights, label_time in predicted_trajectories:
    try:
        max_predicted_height = max(heights)
        relative_error = (max_predicted_height - max_height) / max_height * 100
        error_times.append(label_time)
        relative_errors.append(relative_error)
    except:
        pass

# Plot relative error against time
plt.figure()
plt.plot(error_times, relative_errors, linestyle='-')
plt.xlabel("Time (seconds)")
plt.ylabel("Relative Error")
plt.title("Relative Error vs. Time")
plt.grid(True)
plt.show()