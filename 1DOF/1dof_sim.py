import numpy as np
import matplotlib.pyplot as plt

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
    known_Re = [0, 3.308e6]
    known_Cd = [0.582, 0.638]
    interp_Cd = np.interp(Re, known_Re, known_Cd)
    return interp_Cd

# def cd_ADS(mach):
#     # print(6.95556*mach**2 - 0.618889 * mach + 0.3)
#     return 6.95556*mach**2 - 0.618889 * mach + 0.3
#     # return 1

# def cd_vehicle(density,velocity, height):
#     nu = kinematic_viscosity(height)
#     l = 5.15/144
#     Re = density*velocity*l/nu
#     cd = 0.582 + (0.638 - 0.582) / (3.308e6 - 2.08e6) * (Re - 2.08e6)
#     return cd

### FIX WEIRD LINEAER THING

def ode_solver(ics, properties, dt=0.01):
    # Initialize variables
    current_time = ics.t_0
    current_velocity = ics.v_0
    current_height = ics.h_0
        
    states = [ [], [], [], [], [] ]

    while current_velocity > 0:
        states[0].append(current_time)
        states[1].append(current_height)
        states[2].append(current_velocity)

        T, p, rho, a, nu = fluid_properties(current_height)
        Re = reynolds_number(current_velocity, 5.15/144, nu, rho)
        
        Cd_vehicle = interpolate_cd_vehicle(Re)
        Cd_ads = interpolate_cd_ads(Re)

        F_drag_vehicle = 0.5 * rho * current_velocity**2 * Cd_vehicle * properties.A_vehicle
        F_drag_ADS = 0.5 * rho * current_velocity**2 * Cd_ads * properties.A_ads
        F_gravity = properties.mass * g

        F_net = 0 - F_drag_ADS - F_drag_vehicle - F_gravity

        acceleration_net = F_net / properties.mass

        current_velocity = current_velocity + acceleration_net * dt
        current_height = current_height + current_velocity * dt
        current_time = current_time + dt

    return states

def generic_run():
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

generic_run()
ads_area_comparison_run()
plt.show()

# Simulate for different ADS areas and plot
# for n in range(0,len(A_ads)):
#     times, heights, velocities = compute_apogee(0, h0, v0, mass, A_vehicle, A_ads[n], Cd_ads[n], dt)
#     plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
#     plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=5, ha='left', va='center')