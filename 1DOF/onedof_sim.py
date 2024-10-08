import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
import os
#from scipy.interpolate import griddata

### GLOBAL CONSTANTS IN IMPERIAL UNITS
G = 32.174  # acceleration due to gravity in ft/s^2
MASS = 33.5/G
LREF = 5.15/12
AREF = (5.15**2)*np.pi/4/144

project_root = Path(__file__).parent  # Gets the current directory where the script is located

def fluid_properties_and_constants(altitude, velocity, length):
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

    # Convert temperature from Fahrenheit to Rankine for speed of sound calculation
    T_rankine = T + 459.7
    
    # Speed of sound in ft/s (using specific gas constant for air in imperial units: 1716 ft·lb/(slug·°R))
    a = (1.4 * 1716 * T_rankine)**0.5  

    # Kinematic viscosity in ft^2/s (Sutherland's law adjustment)
    nu = 1.568e-5 * T_rankine / 518.67 * (518.67 + 110.4) / (T + 110.4)

    Re = velocity * length / nu
    M = velocity / a

    return T, p, rho, a, nu, Re, M

def interpolate_cd_ads(Re):
    known_Re = [0, 3.308e6]
    known_Cd = [1.0, 0.97]
    interp_Cd = np.interp(Re, known_Re, known_Cd)
    return interp_Cd

df_openrocket_global = None
poly_func = None
def interpolate_cd_openrocket(Re):
    global df_openrocket_global
    global poly_func
    if df_openrocket_global is None:
        file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
        df_openrocket_global = pd.read_csv(file_path, skiprows=5)
        df_openrocket_global = clean_openrocket_data(df_openrocket_global, 1)
        df_openrocket_global = df_openrocket_global[df_openrocket_global['smoothed_velocity'] > 0]

        Re_data = df_openrocket_global['Reynolds number (​)']
        min_Re = Re_data.min()
        max_Re = Re_data.max()
        Cd_data = df_openrocket_global['Drag coefficient (​)']

    Re_data = df_openrocket_global['Reynolds number (​)']
    min_Re = Re_data.min()
    max_Re = Re_data.max()
    Cd_data = df_openrocket_global['Drag coefficient (​)']
    # # Check if the provided Re is outside the bounds
    if Re < min_Re or Re > max_Re:
        print(f"Warning: Reynolds number {Re} is outside the data range ({min_Re}, {max_Re}).")
        pass
    #have to set the period to np.inf so that it sorts the data
    #np.interp only works for monotomically increasing x so when the data has some 0s it broke
    Cd_interp = np.interp(Re, Re_data, Cd_data,period=np.inf)

    # print(f"Warning: Reynolds number {Re} is Cd {Cd_interp}.")
    return Cd_interp

df_rasaeroii = None
def interpolate_cd_rasaeroii(Re):
    global df_rasaeroii

    if df_rasaeroii is None:
        file_path = project_root / "../rasaeroii_data/0.csv"
        df_rasaeroii = pd.read_csv(file_path)

    Cd = np.interp(Re, df_rasaeroii['Reynolds Number'], df_rasaeroii['CD'])
    return Cd

data_points = None
def interpolate_cd_rasaeroii_map(Re, actuation):
    """Interpolate drag coefficient based on Reynolds number and actuation area."""
    global data_points

    if data_points is None:
        # Get list of all rasaeroii files in the directory ../rasaeroii_data/
        file_path = project_root / "../rasaeroii_data/"
        csv_list = [file for file in os.listdir(file_path) if file.endswith('.CSV')]

        # Initialize data points
        data_points = {}

        # Read each CSV file (ADS_1, ADS_2, etc.)
        for csv in csv_list:
            actuation_area = int(csv.split('.')[0])  # Extract actuation area from file name
            df = pd.read_csv(file_path / Path(csv))

            # Store Reynolds number and CD values for each actuation area
            data_points[actuation_area] = {
                'Reynolds Number': df['Reynolds Number'].values,
                'CD': df['CD'].values
            }

    # Step 1: Interpolate to find Cd values for the known actuation areas using the interpolated Re value
    actuation_areas = np.array(sorted(data_points.keys()))
    
    # Interpolate Cd for the specific Reynolds number at each actuation area
    Cd_values = np.array([np.interp(Re, data_points[area]['Reynolds Number'], data_points[area]['CD']) for area in actuation_areas])

    # Step 2: Interpolate to find the final Cd value at the desired actuation area
    Cd_final = np.interp(actuation, actuation_areas, Cd_values)

    return Cd_final

def interpolate_cd_cfd(Re, M):
    """Interpolate drag coefficient and force based on velocity and altitude."""

    # Read generic data
    file_path = project_root / "../cfd_data/generic.csv"
    df = pd.read_csv(file_path)

    # Interpolate Cd based on Re or M for incompressible vs compressible flow
    if M < 0.3: # Incompressible flow
        Re_column = df[df.columns[1]]
        Cd_column = df['Cd']
        Cd = np.interp(Re, Re_column, Cd_column)
    else: # Compressible flow
        M_column = df[df.columns[0]]
        Cd_column = df['Cd']
        Cd = np.interp(M, M_column, Cd_column)

    # Compute drag force
    # F_drag = 0.5 * rho * v**2 * Cd * 5.15/144

    return Cd

def ode_solver(ics, properties, cd_handle="openrocket", dt=0.01):
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
        T, p, rho, a, nu, Re, M = fluid_properties_and_constants(current_height, current_velocity, LREF)
        
        # Lookup drag coefficients
        # Cd_vehicle = interpolate_cd_vehicle(Re)
        if cd_handle == 'rasaeroii':
            Cd_vehicle = interpolate_cd_rasaeroii(Re)
        elif cd_handle == 'openrocket':
            Cd_vehicle = interpolate_cd_openrocket(Re)
        elif cd_handle == 'cfd':
            Cd_vehicle = interpolate_cd_cfd(Re, M)
        Cd_ads = interpolate_cd_ads(Re)

        # Compute forces
        F_drag_vehicle = 0.5 * rho * current_velocity**2 * Cd_vehicle * properties.A_vehicle
        F_drag_ADS = 0.5 * rho * current_velocity**2 * Cd_ads * properties.A_ads
        F_gravity = properties.mass * G

        # Apply F = ma
        F_net = 0 - F_drag_ADS - F_drag_vehicle - F_gravity
        acceleration_net = F_net / properties.mass

        # Update states
        current_velocity = current_velocity + acceleration_net * dt
        current_height = current_height + current_velocity * dt
        current_time = current_time + dt

    return states

def generic_run():
    """Simulate the rocket for a generic without ADS and compare to OpenRocket."""
    file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
    df = pd.read_csv(file_path, skiprows=5)
    df_clean = clean_openrocket_data(df, 1)

    # Get rid of this magic number
    df_clean = df_clean[df_clean['time'] > 3.141]

    # Get rid of this magic number too
    ics = type('ics', (object,), {'t_0': 3.141, 'v_0': 646.342, 'h_0': 1144.009})
    properties = type('properties', (object,), {'mass': MASS, 'A_vehicle': AREF, 'A_ads': 0})
    states_openrocket = ode_solver(ics, properties, cd_handle='openrocket')
    states_rasaeroii = ode_solver(ics, properties, cd_handle='rasaeroii')
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(states_openrocket[0], states_openrocket[1], label='1DOF -- OpenRocket Cd')
    ax1.plot(states_rasaeroii[0], states_rasaeroii[1], label='1DOF -- RasAeroII Cd')
    openrocket_time = df_clean['time'][df_clean['smoothed_velocity'] > 0]
    openrocket_altitude = df_clean['smoothed_altitude'][df_clean['smoothed_velocity'] > 0]
    ax1.plot(openrocket_time, openrocket_altitude, label='CSV -- OpenRocket Data', linestyle='--')

    simulated_altitude_interpolated = np.interp(openrocket_time, states_openrocket[0], states_openrocket[1])

    # Compute the absolute error between OpenRocket and simulated altitudes
    error = np.abs(openrocket_altitude - simulated_altitude_interpolated)

    ax2.plot(openrocket_time, error, label='Error', linestyle=':', color='r')
    ax1.set_title(f'Rocket Altitude vs Time -- h0: {ics.h_0} ft, v0: {ics.v_0} ft/s')
    
    ax1.set_ylabel('Altitude [ft]')
    ax1.legend(loc='upper left')
    ax1.grid(True)

    # Customize the second subplot (Error)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('∆ (OpenRocket - 1DOF Sim) [ft]')
    ax2.legend(loc='upper left')
    ax2.grid(True)

    plt.tight_layout()

def ads_area_comparison_run():
    """Simulate the rocket for different ADS areas and plot the results."""
    ads_areas = [0, 1, 2, 3, 4, 5, 6, 24]
    ics = type('ics', (object,), {'t_0': 3.141, 'v_0': 646.342, 'h_0': 1144.009})
    fig, ax1 = plt.subplots(figsize=(10, 6))
    for ads_area in ads_areas:
        properties = type(' properties', (object,), {'mass': MASS, 'A_vehicle': AREF, 'A_ads': ads_area/144})
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
    # Uses OpenRocket trajectory data to compare the effect of ADS activation on apogee
    file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
    df = pd.read_csv(file_path, skiprows=5)
    df_clean = clean_openrocket_data(df, 1)

    # Get rid of this magic number
    df_clean = df_clean[df_clean['time'] > 3]

    total_rows = len(df)
    N = 100
    
    step_size = max(total_rows // N, 1)
    start_time = []
    all_trajectories_active = []
    all_trajectories_inactive = []
    properties_active = type('properties', (object,), {'mass': MASS, 'A_vehicle': AREF, 'A_ads': 7/144})
    properties_inactive = type('properties', (object,), {'mass': MASS, 'A_vehicle': AREF, 'A_ads': 0})
    
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
    ax2 = ax1.twinx()
    
    for index in range(len(all_trajectories_active)):
        try:
            # active_apogee = max(all_trajectories_active[index][1])
            # inactive_apogee = max(all_trajectories_inactive[index][1])
            delta = max(all_trajectories_inactive[index][1]) - max(all_trajectories_active[index][1])
            # find the time where this trajectory started
            # get the velocity at that time
            ax1.plot(start_time[index], delta, 'ro')

            velocity_at_ads = np.interp(start_time[index], df_clean['time'], df_clean['smoothed_velocity'])
            ax2.plot(start_time[index], velocity_at_ads, 'bo')

        except Exception as e:
            # print(f"Error processing row {index}: {e}")
            pass

    # Set plot titles and labels
    ax1.set_title("Various Trajectories where ADS is Started at T+X")
    ax1.set_xlabel("T+XX where ADS Activates 100% [s]")
    ax1.set_ylabel("Delta [ft]")
    ax2.set_ylabel("Velocity at ADS Activation [ft/s]")
    ax1.grid(True)

def plot_ahmads_cdf():
    # Read generic data
    file_path = project_root / "../cfd_data/cfd_incompressible_base.csv"
    df = pd.read_csv(file_path, skiprows=6)
    df["Cd"] = (df["P3"] / 4.448) / (0.5 * df["P1"] * df["P2"]**2 * AREF)
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(df['P4'], df['Cd'], label='Incompressible Flow')
    ax1.set_title("Drag Coefficient vs Reynolds Number")
    ax1.set_xlabel("Reynolds Number")
    ax1.set_ylabel("Drag Coefficient")
    ax1.legend()
    ax1.grid(True)

def cd_comparison_pure_data():
    fig, ax1 = plt.subplots(figsize=(10, 6))

    file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
    df_openrocket = pd.read_csv(file_path, skiprows=5)
    df_openrocket = clean_openrocket_data(df_openrocket, 1)
    df_openrocket = df_openrocket[df_openrocket['smoothed_velocity'] > 0]

    Re = df_openrocket['Reynolds number (​)']
    Cd = df_openrocket['Drag coefficient (​)']

    ax1.plot(Re, Cd, 'ro', label='OpenRocket Pure Data')

    # file_path = project_root / "../rasaeroii_data/0.CSV"
    # df_rasaeroii = pd.read_csv(file_path)
    # Re = df_rasaeroii['Reynolds Number']
    # Cd = df_rasaeroii['CD']
    # ax1.plot(Re, Cd, 'b-', label='RasAeroII Pure Data')

    # file_path = project_root / "../cfd_data/cfd_incompressible_base.csv"
    # df = pd.read_csv(file_path, skiprows=6)
    # df["Cd"] = (df["P3"] / 4.448) / (0.5 * df["P1"] * df["P2"]**2 * AREF)
    # ax1.plot(df['P4'], df['Cd'], label='Incompressible Flow')

    Re_range = np.linspace(0, 5e6, 50)

    Cd_openrocket = [interpolate_cd_openrocket(Re) for Re in Re_range]
    ax1.plot(Re_range, Cd_openrocket, label='OpenRocket Interpolation')

    ax1.set_xlim([0, 5e6])
    ax1.set_title("Drag Coefficient vs Reynolds Number")
    ax1.set_xlabel("Reynolds Number")
    ax1.set_ylabel("Drag Coefficient")
    ax1.legend()
    ax1.grid(True)


def cd_comparison():
    # Compare the Cd values computed from various sources
    Re_range = np.linspace(0, 5e6, 50)
    fig, ax1 = plt.subplots(figsize=(10, 6))
    Cd_openrocket = [interpolate_cd_openrocket(Re) for Re in Re_range]
    ax1.plot(Re_range, Cd_openrocket, label='OpenRocket Interpolation')

    file_path = project_root / "../openrocket_data/Disturbance_Sims_Base.csv"
    df_openrocket = pd.read_csv(file_path, skiprows=5)
    df_openrocket = clean_openrocket_data(df_openrocket, 1)
    df_openrocket = df_openrocket[df_openrocket['smoothed_velocity'] > 0]

    Re = df_openrocket['Reynolds number (​)']
    Cd = df_openrocket['Drag coefficient (​)']
    ax1.plot(Re, Cd, 'r-', label='OpenRocket Pure Data')
    ax1.set_title("Drag Coefficient vs Reynolds Number")
        
    for i in np.linspace(0, 1, num=1):  # 1 to 7 with 0.5 intervals
        Cd_map = [interpolate_cd_rasaeroii_map(Re, i) for Re in Re_range]
        ax1.plot(Re_range, Cd_map, label=f'RasAeroII Map -- {i:.1f} in²')

    file_path = project_root / "../cfd_data/cfd_incompressible_base.csv"
    df = pd.read_csv(file_path, skiprows=6)
    df["Cd"] = (df["P3"] / 4.448) / (0.5 * df["P1"] * df["P2"]**2 * AREF)
    ax1.plot(df['P4'], df['Cd'], label='Incompressible Flow')

    ax1.set_xlim([0, 5e6])
    ax1.set_title("Drag Coefficient vs Reynolds Number")
    ax1.set_xlabel("Reynolds Number")
    ax1.set_ylabel("Drag Coefficient")
    ax1.legend()
    ax1.grid(True)

# generic_run()
# ads_area_comparison_run()
# not_trajectories()
cd_comparison()
# cd_comparison_pure_data()

print("All plots except the cd comparison one use openrocket Cd data. The last plot compares Cd values from different sources.")

# plot_ahmads_cdf()

plt.show()