import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from pathlib import Path

# SOMETHING IS VERY BROKEN IN MY CODE
# I WILL WORK ON IT

# Constants
A_vehicle = 24.581150 / 144  # Reference area of the vehicle 
A_ads_mach = 9.18 / 144
Cd_ads = 1.0
mass = 0.932429 + 0.1086755  # Mass of the rocket

def mach_helper(height_ft, velocity):
    Tk = (518.67 - 0.003566 * height_ft) * 5 / 9  # Temp in Kelvins
    speed_of_sound = ((1.4 * 8.314 * Tk / 0.02896) ** 0.5)
    mach = (velocity / 3.281) / speed_of_sound
    return mach

# get rid of these magic numbers
def cd_ADS(mach):
    return 6.95556 * mach**2 - 0.618889 * mach + 0.3

def drag_force_ADS(rho, v, A, Cd):
    return 0.5 * rho * v**2 * Cd * A

def density(h):
    """Air density based on altitude (ft)."""
    if h < 36152:
        T = 59 - 0.00356 * h  # Temperature in Fahrenheit
        p = 2116 * ((T + 459.7) / 518.6) ** 5.256  # Pressure in lbf/ft^2
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

# get rid of these magic numbers
def cd_vehicle(density,velocity, height):
    nu = kinematic_viscosity(height)
    l = 5.15/144
    Re = density*velocity*l/nu
    cd = 0.582 + (0.638 - 0.582) / (3.308e6 - 2.08e6) * (Re - 2.08e6)
    return cd 

def compute_apogee_mach_func(time_in, height_in, velocity_in, mass, A_vehicle, dt, N):
    # Initialize variables
    current_velocity = velocity_in
    current_height = height_in
    current_time = time_in

    g = 32.174  # ft/s^2
    rho_air = density(height_in)
    Cd_vehicle = cd_vehicle(rho_air, current_velocity, current_height)
    times, heights, velocities = [], [], []

    count = 0

    while current_velocity > 0 and count < N:  # limit loop by N initial conditions
        dt = 0.01
        times.append(current_time)
        mach = mach_helper(current_height, current_velocity)
        F_drag = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle + drag_force_ADS(rho_air, current_velocity, A_ads_mach, cd_ADS(mach))
        F_gravity = mass * g
        F_net = -F_drag - F_gravity
        acceleration_net = F_net / mass
        current_velocity += acceleration_net * dt
        current_height += current_velocity * dt
        current_time += dt
        prev_velocity = current_velocity
        heights.append(current_height)
        velocities.append(current_velocity)
        count += 1

    return times, heights, velocities

# Load data from CSV and select N evenly spaced initial conditions
def load_and_simulate(file_name, N):
    df = pd.read_csv(file_name, on_bad_lines='skip')
    total_rows = len(df)
    
    # Calculate step size based on N
    step_size = max(total_rows // N, 1)  # Ensure step size is at least 1
    
    predicted_trajectories_mach = []
    
    # Iterate through evenly spaced rows
    for index in range(0, total_rows, step_size):
        if len(predicted_trajectories_mach) >= N:
            break  # Stop after collecting N conditions
        
        row = df.iloc[index]
        ic_time = row['time']
        ic_height = row['altitude']
        ic_velocity = row['speed']

        if ic_time > 2.129 and ic_time < 17.379:
            times, heights, velocities = compute_apogee_mach_func(ic_time, ic_height, ic_velocity, mass, A_vehicle, 0.01, N)
            predicted_trajectories_mach.append((times, heights, ic_time))
    
    ic_times_mach = []
    apogees_mach = []
    for trajectories in predicted_trajectories_mach:
        ic_times_mach.append(trajectories[2])
        apogees_mach.append(max(trajectories[1]))

    max_apogee = df['altitude'].max()

    return ic_times_mach, apogees_mach, max_apogee

# File and number of initial conditions
project_root = Path(__file__).parent  # Gets the current directory where the script is located
file_path = project_root / "data/openrocket_v3.csv"

N = 100  # Specify how many evenly spaced initial conditions to use
ic_times_mach, apogees_mach, max_apogee = load_and_simulate(file_path, N)

# Plot results
plt.figure()
plt.plot(ic_times_mach, apogees_mach, label=f'ADS Area = {round(A_ads_mach * 144, 2)} in²')
plt.title("Apogee vs Actuation Time")
plt.xlabel("T+X ADS Starts Actuating [s]")
plt.ylabel("Apogee [ft]")
plt.grid(True)
plt.legend()
plt.show()