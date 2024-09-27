import numpy as np
import matplotlib.pyplot as plt

# Constants
A_vehicle = 24.581150/144  # Reference area of the vehicle 
A_ads = [0, 3/144, 4/144, 5/144, 6/144, 24/144] # ADS areas
Cd_ads = [1.0, 0.55, 0.55, 0.55, 0.7, 0.97]  # Drag coefficient of ADS (flat plate)

# Initial conditions
h0 = 738.2  #Initial height at burnout (ft)
v0 = 653.2  # Initial velocity at burnout (ft/s)
mass = 0.90  # Mass of the rocket
dt = 0.1

# Function to calculate drag force for ADS
def drag_force_ADS(rho, v, A, Cd):
    return 0.5 * rho * v**2 * Cd * A

def density(h):
    """Air density based on altitude (ft)."""
    if h < 36152:
        T = 59 - 0.00356 * h  # Temperature in Fahrenheit
        p = 2116 * ((T + 459.7) / 518.6)**5.256  # Pressure in lbf/ft^2
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    elif h < 82345:
        T = -70
        p = 473.1 * np.exp(1.73 - 0.000048 * h)
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    else:
        rho = -1
    return rho

def compute_apogee(time_in, height_in, velocity_in, mass, A_vehicle, A_ads, Cd_ads, dt):
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = density(height_in)  # density of air in slugs/ft^3
    Cd_vehicle = 0.6  # Drag coefficient of the vehicle
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
        F_drag = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle + drag_force_ADS(rho_air, current_velocity, A_ads, Cd_ads)
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

# Simulate for different ADS areas and plot
plt.figure(figsize=(10, 8))

for n in range(0,len(A_ads)):
    times, heights, velocities = compute_apogee(0, h0, v0, mass, A_vehicle, A_ads[n], Cd_ads[n], dt)
    plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
    plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=5, ha='left', va='center')



# Plot settings
plt.title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {h0}ft, v0: {v0}ft/s")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (ft)")
plt.legend()
plt.grid(True)
plt.show()