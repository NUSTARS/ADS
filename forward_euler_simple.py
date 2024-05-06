import matplotlib.pyplot as plt

def compute_apogee(height_in, velocity_in, acceleration_in, mass, area, dt):
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = 0.002378  # density of air in slugs/ft^3
    Cd = 0.4  # drag coefficient
    
    time_step = dt  # time step for Euler integration
    
    # Initialize variables
    current_velocity = velocity_in
    current_height = height_in
    current_time = 0
    
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

    # return current_height


# Example usage
current_height = 0  # Initial height in feet
velocity = 550  # Initial velocity in ft/s
acceleration = -32.17  # Constant acceleration (assuming downwards as negative) in ft/s^2
mass = 37.15/32.17  # Mass of the rocket in slugs (1 slug = 32.174 lbs·s^2/ft)
area = 0.25  # Cross-sectional area of the rocket in ft^2

# apogee = compute_apogee(current_height, velocity, acceleration, mass, area, 0.01)
# print("Apogee reached:", apogee, "feet")

dt_values = [0.01, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]  # List of different time steps

# Plot trajectories for each variation
for dt in dt_values:
    times, heights, velocities = compute_apogee(current_height, velocity, acceleration, mass, area, dt)
    plt.plot(times, heights, label=f"dt={dt}")
    
plt.xlabel("Time (seconds)")
plt.ylabel("Height (feet)")
plt.title("Rocket Trajectories with Different Time Steps")
plt.legend()
plt.grid(True)
plt.show()
