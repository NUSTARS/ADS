import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os

### VEHICLE PROPERTIES
# A_vehicle = 24.581150/144 # Area of the vehicle [ft^2]
# A_ads = 6/144 # Area of the ADS [ft^2]
# mass = 0.90  # Mass of the rocket [slugs]
# l = 5.15/144 # Reference length of the rocket [ft]
# A_ads_mach = 6.68/144
# Cd_ads = [1.0, 0.55, 0.55, 0.55, 0.7, 0.97]  # Drag coefficient of ADS (flat plate)

# Initial conditions
h0 = 1000  #Initial height at burnout [ft]
v0 = 656.6  # Initial velocity at burnout [ft/s]
dt = 0.01 # Time step for Euler integration [s]

### GLOBAL CONSTANTS IN IMPERIAL UNITS
g = 32.174  # acceleration due to gravity in ft/s^2

project_root = Path(__file__).parent  # Gets the current directory where the script is located

def fluid_properties(altitude):
    """Air density based on altitude (ft)."""
    if altitude < 36152:
        T = 59 - 0.00356 * altitude  # Temperature in Fahrenheit
        p = 2116 * ((T + 459.7) / 518.6)**5.256  # Pressure in lbf/ft^2
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    elif altitude < 82345:
        T = -70
        p = 473.1 * np.exp(1.73 - 0.000048 * altitude)
        rho = p / (1718 * (T + 459.7))  # Density in slugs/ft^3
    else:
        rho = -1

    a = (1.4 * 8.314 * (T + 459.7) / 0.02896)**0.5  # Speed of sound in ft/s

    nu = 1.568e-5 * (T + 459.7) / 518.67 * (518.67 + 110.4) / (T + 110.4)  # Kinematic viscosity in ft^2/s

    return T, p, rho, a, nu

def reynolds_number(velocity, length, nu, rho):
    """Calculate Reynolds number based on velocity, characteristic length, kinematic viscosity, and density."""
    Re = rho * velocity * length / nu
    return Re

def interpolate_cd_ads(Re):
    known_Re = [0, 3.308e6]
    known_Cd = [1.0, 0.97]
    interp_Cd = np.interp(Re, known_Re, known_Cd)
    return interp_Cd

def interpolate_cd_vehicle(Re):
    known_Re = [2.08e6, 3.308e6]
    known_Cd = [0.582, 0.638]
    interp_Cd = np.interp(Re, known_Re, known_Cd)
    return interp_Cd

# def cd_ADS(mach):
#     # print(6.95556*mach**2 - 0.618889 * mach + 0.3)
#     return 6.95556*mach**2 - 0.618889 * mach + 0.3
#     # return 1

def ode_solver(ics, properties, dt=0.01):
    """Solve the 1DOF equations of motion using Euler integration."""

    # Initialize variables
    current_time = ics.t_0
    current_velocity = ics.v_0
    current_height = ics.h_0        
    states = [ [], [], [], [], [] ]

    # Only simulate while the rocket is moving upwards
    while current_velocity > 0:
        # Get current states
        states[0].append(current_time)
        states[1].append(current_height)
        states[2].append(current_velocity)

        # Compute altitude-dependent fluid properties
        T, p, rho, a, nu = fluid_properties(current_height)
        Re = reynolds_number(current_velocity, 5.15/144, nu, rho)
        
        # Lookup drag coefficients
        Cd_vehicle = interpolate_cd_vehicle(Re)
        Cd_ads = interpolate_cd_ads(Re)

        # Compute forces
        F_drag_vehicle = 0.5 * rho * current_velocity**2 * Cd_vehicle * properties.A_vehicle
        F_drag_ADS = 0.5 * rho * current_velocity**2 * Cd_ads * properties.A_ads
        F_gravity = properties.mass * g

        # Apply F = ma
        F_net = 0 - F_drag_ADS - F_drag_vehicle - F_gravity
        acceleration_net = F_net / properties.mass

        # Update states
        current_velocity = current_velocity + acceleration_net * dt
        current_height = current_height + current_velocity * dt
        current_time = current_time + dt

    return states

def generic_run():
    """Simulate the rocket for a generic without ADS."""
    ics = type('ics', (object,), {'t_0': 2.8, 'v_0': 656.6, 'h_0': 1000})
    properties = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 0})
    states = ode_solver(ics, properties)
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Use properties.A_ads to refer to the ADS area
    ax1.plot(states[0], states[1], label=f'ADS Area = {(properties.A_ads)*144:.2f} in²')
    
    # Set the title, labels, legend, and grid
    ax1.set_title("Rocket Altitude vs Time -- h0: 1000 ft, v0: 656.6 ft/s")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Altitude [ft]")
    ax1.legend()
    ax1.grid(True)

def ads_area_comparison_run():
    """Simulate the rocket for different ADS areas and plot the results."""
    ads_areas = [0.5, 1, 2, 3, 4, 5, 6]
    ics = type('ics', (object,), {'t_0': 2.8, 'v_0': 656.6, 'h_0': 1000})
    fig, ax1 = plt.subplots(figsize=(10, 6))
    for ads_area in ads_areas:
        properties = type(' properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': ads_area/144})
        states = ode_solver(ics, properties)
        ax1.plot(states[0], states[1], label=f'ADS Area = {ads_area} in²')
    ax1.set_title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {h0} ft, v0: {v0} ft/s")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Altitude [ft]")
    ax1.legend()
    ax1.grid(True)

def clean_openrocket_data(df, alpha):
    assert 0 <= alpha <= 1, 'Alpha must be between 0 and 1'

    # Skip rows with event information based on the '#' identifier in the 'Time' column
    df = df[~df['# Time (s)'].astype(str).str.startswith('#')]

    # Convert the 'Time' column to integers
    df["time"] = df['# Time (s)'].astype(float)

    # Apply Exponential Weighted Moving Average (EWMA) smoothing
    df["smoothed_altitude"] = df['Altitude (ft)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_velocity"] = df['Vertical velocity (ft/s)'].ewm(alpha=alpha, min_periods=1).mean()
    df["smoothed_acceleration"] = df['Vertical acceleration (ft/s²)'].ewm(alpha=alpha, min_periods=1).mean()

    return df

def not_trajectories():
    properties = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 0})
    
    file_path = project_root / "../openrocket_data/Disturbance_Sims_Pressure.csv"
    df = pd.read_csv(file_path, skiprows=5)
    df_clean = clean_openrocket_data(df, 1)
    total_rows = len(df)
    N = 100
    
    step_size = max(total_rows // N, 1)
    
    all_tracjectories = []

    properties = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 6/144})
    
    # Iterate through evenly spaced rows
    for index in range(0, total_rows, step_size):
        if index > total_rows:
            return
        row = df_clean.iloc[index]
        ic = type('ics', (object,), {'t_0': row['time'], 'v_0': row['smoothed_velocity'], 'h_0': row['smoothed_altitude']})
        states = ode_solver(ic, properties, dt=0.01)
        all_tracjectories.append(states)
        
    return all_tracjectories

#     # Plot results
#     plt.figure()
#     # plot the maximum apogees
#     # plt.plot(ic_times_mach, apogees_mach, label=f'ADS Area = {round(A_ads_mach * 144, 2)} in²')
#     plt.title("Apogee vs Actuation Time")
#     plt.xlabel("T+X ADS Starts Actuating [s]")
#     plt.ylabel("Apogee [ft]")
#     plt.grid(True)
#     plt.legend()
#     plt.show()

# generic_run()
# ads_area_comparison_run()
not_trajectories()

# plt.show()