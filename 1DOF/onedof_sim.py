import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os

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

def interpolate_cd_general(v, h):
    """Interpolate drag coefficient and force based on velocity and altitude."""

    # Assume reference area and length dependent ONLY on vehicle outer diameter

    # Calculate Reynolds number and Mach number
    T, p, rho, a, nu = fluid_properties(h)
    Re_local = reynolds_number(v, 5.15/144, nu, rho)
    M_local = v / a

    # Read generic data
    file_path = project_root / "../cfd_data/generic.csv"
    df = pd.read_csv(file_path)

    # Interpolate Cd based on Re or M for incompressible vs compressible flow
    if M_local < 0.3: # Incompressible flow
        Re_column = df[df.columns[1]]
        Cd_column = df['Cd']
        Cd = np.interp(Re_local, Re_column, Cd_column)
    else: # Compressible flow
        M_column = df[df.columns[0]]
        Cd_column = df['Cd']
        Cd = np.interp(M_local, M_column, Cd_column)

    # Compute drag force
    F_drag = 0.5 * rho * v**2 * Cd * 5.15/144

    return Cd, F_drag

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
    ax1.set_title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {ics.h_0} ft, v0: {ics.v_0} ft/s")
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
    file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
    df = pd.read_csv(file_path, skiprows=5)
    df_clean = clean_openrocket_data(df, 1)

    # Get rid of this magic number
    df_clean = df_clean[df_clean['time'] > 2.88]

    total_rows = len(df)
    N = 100
    
    step_size = max(total_rows // N, 1)
    start_time = []
    all_trajectories_active = []
    all_trajectories_inactive = []
    properties_active = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 6.68/144})
    properties_inactive = type('properties', (object,), {'mass': 0.90, 'A_vehicle': 24.581150/144, 'A_ads': 0})
    
    for index in range(0, total_rows, step_size):
        try:
            row = df_clean.iloc[index]

            start_time.append(row['time'])
            
            ic = type('ics', (object,), {'t_0': row['time'], 'v_0': row['smoothed_velocity'], 'h_0': row['smoothed_altitude']})
            states_active = ode_solver(ic, properties_active, dt=0.01)
            all_trajectories_active.append(states_active)
            
            states_inactive = ode_solver(ic, properties_inactive, dt=0.01)
            all_trajectories_inactive.append(states_inactive)
        except Exception as e:
            # print(f"Error processing row {index}: {e}")
            pass
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    for index in range(len(all_trajectories_active)):
        try:
            active_apogee = max(all_trajectories_active[index][1])
            inactive_apogee = max(all_trajectories_inactive[index][1])
            delta = max(all_trajectories_inactive[index][1]) - max(all_trajectories_active[index][1])
            ax1.plot(start_time[index], delta, 'ro')

        except Exception as e:
            # print(f"Error processing row {index}: {e}")
            pass

    # Set plot titles and labels
    ax1.set_title("Various Trajectories where ADS is Started at T+X")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Delta [ft]")
    ax1.legend() # Display a legend
    ax1.grid(True)

generic_run()
ads_area_comparison_run()
not_trajectories()

plt.show()