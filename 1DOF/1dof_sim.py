import numpy as np
import matplotlib.pyplot as plt

# Constants
A_vehicle = 24.581150/144  # Reference area of the vehicle [ft^2]
A_ads = [0, 3/144, 4/144, 5/144, 6/144, 24/144] # ADS areas [ft^2]
Cd_ads = [1.0, 0.55, 0.55, 0.55, 0.7, 0.97]  # Drag coefficient of ADS (flat plate)

A_ads_mach = 6.68/144

# Initial conditions
h0 = 1000  #Initial height at burnout [ft]
v0 = 656.6  # Initial velocity at burnout [ft/s]
mass = 0.90  # Mass of the rocket [slugs]
dt = 0.1

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

def compute_apogee_mach_func(time_in, height_in, velocity_in, mass, A_vehicle, A_ads, dt):
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
        F_drag_vehicle = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle
        Cd_ADS = cd_ADS(mach_helper(current_height, current_velocity))
        F_drag_ADS = 0.5 * rho_air * current_velocity**2 * Cd_ADS * A_ads_mach
        F_gravity = mass * g
        F_net = -1 * F_drag_ADS - F_drag_vehicle - F_gravity
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
plt.figure(figsize=(10, 8))


times, heights, velocities = compute_apogee_mach_func(0, h0, v0, mass, A_vehicle, A_ads, dt)
plt.plot(times, heights, label=f'ADS Area = {(A_ads_mach)*144} in²')
plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=10, ha='left', va='center')


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
        F_drag_vehicle = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle
        Cd_ADS = cd_ADS(mach_helper(current_height, current_velocity))
        F_drag_ADS = 0.5 * rho_air * current_velocity**2 * Cd_ADS * A_ads_mach
        F_gravity = mass * g
        F_net = -1 * F_drag_ADS - F_drag_vehicle - F_gravity
        acceleration_net = F_net / mass
        current_velocity = current_velocity + acceleration_net * time_step
        current_height = current_height + current_velocity * time_step
        current_time += time_step
        
        # Save current height and velocity
        heights.append(current_height)
        velocities.append(current_velocity)

    return times, heights, velocities

# Simulate for different ADS areas and plot

for n in range(0,len(A_ads)):
    times, heights, velocities = compute_apogee(0, h0, v0, mass, A_vehicle, A_ads[n], Cd_ads[n], dt)
    plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
    plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=5, ha='left', va='center')

# plt.plot(times, mach_nums)
# Plot settings
plt.title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {h0}ft, v0: {v0}ft/s")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (ft)")
plt.legend()
plt.grid(True)
plt.show()