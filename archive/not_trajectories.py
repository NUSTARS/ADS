import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from pathlib import Path

# Constants
A_vehicle = 24.581150/144  # Reference area of the vehicle 
A_ads = 0
Cd_ads = 1.0
# A_ads = [0, 3/144, 4/144, 5/144, 6/144, 24/144] # ADS areas
# Cd_ads = [1.0, 0.55, 0.55, 0.55, 0.7, 0.97]  # Drag coefficient of ADS (flat plate)
# A_ads = [ 6.4/144, 6.5/144, 6.6/144] # ADS areas
# Cd_ads = [1.28, 1.28, 1.28]
A_ads_mach = 9.18/144

# Initial conditions
# h0 = 1000  #Initial height at burnout (ft)
# v0 = 656.6  # Initial velocity at burnout (ft/s)
mass = 0.932429 + 0.1086755  # Mass of the rocket
dt = 0.01

def mach_helper (height_ft, velocity):
    #assumptions: same temp as in kinematic_viscosity, air adiabatic index constant, air chemistry constant
    Tk = (518.67 - 0.003566 * height_ft) * 5/9 #temp in Kelvins
    
    speed_of_sound = ((1.4*8.314*Tk/0.02896)**0.5)
    # print(speed_of_sound)
    mach = (velocity/3.281)/speed_of_sound
    # print('mach')
    # print(mach)
    return mach


def cd_ADS(mach):
    # print(6.95556*mach**2 - 0.618889 * mach + 0.3)
    return 6.95556*mach**2 - 0.618889 * mach + 0.3
    # return 1


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

def kinematic_viscosity(height_ft):
    # Constants for Sutherland's formula
    nu_0 = 1.568e-5  # kinematic viscosity at sea level (ft^2/s)
    T0 = 518.67  # reference temperature at sea level (R)
    S = 110.4  # Sutherland's constant (R)
    # Calculate temperature at the given height (assuming standard lapse rate)
    T = 518.67 - 0.003566 * height_ft  # standard lapse rate: 0.003566 R/ft
    # Calculate kinematic viscosity using Sutherland's formula
    nu = nu_0 * (T / T0)**(3/2) * (T0 + S) / (T + S)
    return nu

def cd_vehicle(density,velocity, height):
    nu = kinematic_viscosity(height)
    l = 5.15/144
    Re = density*velocity*l/nu
    cd = 0.582 + (0.638 - 0.582) / (3.308e6 - 2.08e6) * (Re - 2.08e6)
    return cd 

def compute_apogee(time_in, height_in, velocity_in, mass, A_vehicle, A_ads, Cd_ads, dt):
    # Initialize variables
    current_velocity = velocity_in
    current_height = height_in
    current_time = time_in
    
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = density(height_in)  # density of air in slugs/ft^3
    Cd_vehicle = cd_vehicle(rho_air,current_velocity, current_height)  # Drag coefficient of the vehicle
    time_step = dt  # time step for Euler integration
    
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

def compute_apogee_mach_func(time_in, height_in, velocity_in, mass, A_vehicle, dt):
    # Initialize variables
    current_velocity = velocity_in
    current_height = height_in
    current_time = time_in
    
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = density(height_in)  # density of air in slugs/ft^3
    Cd_vehicle = cd_vehicle(rho_air,current_velocity, current_height)  # Drag coefficient of the vehicle
    time_step = dt  # time step for Euler integration
    
    
    times = []
    heights = []
    velocities = []
    
    
    while current_velocity > 0:
        times.append(current_time)
        F_drag = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle + drag_force_ADS(rho_air, current_velocity, A_ads_mach, cd_ADS(mach_helper(current_height, current_velocity)))
        F_gravity = mass * g
        F_net = -1 * F_drag - F_gravity
        acceleration_net = F_net / mass
        current_velocity = current_velocity + acceleration_net * time_step
        current_height = current_height + current_velocity * time_step
        current_time += time_step
        # print(mach_helper(current_height, current_velocity))
        
        # Save current height and velocity
        heights.append(current_height)
        velocities.append(current_velocity)

    return times, heights, velocities



# Simulate for different ADS areas and plot
# plt.figure(figsize=(10, 8))
plt.figure()

#reads cs
project_root = Path(__file__).parent  # Gets the current directory where the script is located
file_path = project_root / "data/openrocket_v3.csv"
df = pd.read_csv(file_path, on_bad_lines='skip')

predicted_trajectories_mach = []
    #loops through every line of the csv and uses it as ics
for index, row in df.iterrows():
    ic_time = row['time']
    ic_height = row['altitude']  # Initial height in feet
    ic_velocity = row['speed']  # Initial velocity in ft/s

        # # Compute trajectory for this time step as long as the rocket is 1. no longer in boost (a < 0) and 2. not falling (v > 0)
    if ic_time > 2.129 and ic_time < 17.379:
            # print(ic_time)
        times, heights, velocities = compute_apogee_mach_func(ic_time, ic_height, ic_velocity, mass, A_vehicle, dt)
        label_time = ic_time

        # Append predicted trajectory data to the list
        predicted_trajectories_mach.append((times, heights, label_time))
    ic_times_mach = []
    apogees_mach = []
    for trajectories in predicted_trajectories_mach:
        ic_times_mach.append(trajectories[2])
        apogees_mach.append(max(trajectories[1]))
    
# plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
# print(apogees_mach)
# plt.plot(ic_times_mach, apogees_mach, linewidth = 2.5, label=f'ADS Area = {round(A_ads_mach*144, 2)} in²')

#--------------------------the following is for calculating and plotting delta----------------------------------

predicted_trajectories_zero = []
    #loops through every line of the csv and uses it as ics
for index, row in df.iterrows():
    ic_time = row['time']
    ic_height = row['altitude']  # Initial height in feet
    ic_velocity = row['speed']  # Initial velocity in ft/s

        # # Compute trajectory for this time step as long as the rocket is 1. no longer in boost (a < 0) and 2. not falling (v > 0)
    if ic_time > 2.129 and ic_time < 17.379:
            # print(ic_time)
        times, heights, velocities = compute_apogee(ic_time, ic_height, ic_velocity, mass, A_vehicle, A_ads, Cd_ads, dt)
        label_time = ic_time

        # Append predicted trajectory data to the list
        predicted_trajectories_zero.append((times, heights, label_time))
    ic_times_zero = []
    apogees_zero = []
    for trajectories in predicted_trajectories_zero:
        ic_times_zero.append(trajectories[2])
        apogees_zero.append(max(trajectories[1]))
    
# plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
# plt.plot(ic_times_zero, apogees_zero, linewidth = 1, label=f'ADS Area = {round(A_ads_mach*144, 2)} in²')

apogees_delta = np.subtract(apogees_zero, apogees_mach)
print(apogees_delta)

plt.plot(ic_times_zero, apogees_delta, linewidth = 1,)

#--------------------------the previous is for calculating and plotting delta----------------------------------

# Plot settings
plt.title("Apogee vs Actuation Time, starting from burnout (0 in - 6.88 in)")
plt.xlabel("Time That ADS Actuates (s)")
plt.ylabel("Delta Apogee (ft)")
plt.legend()
plt.grid(True)
plt.show()


