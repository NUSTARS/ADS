import numpy as np
import matplotlib.pyplot as plt

### VEHICLE PROPERTIES
A_vehicle = 25/144
A_ads = 6/144
mass = 0.90  # Mass of the rocket [slugs]
l = 5.15/144
# A_ads_mach = 6.68/144

# Initial conditions
h0 = 1000  #Initial height at burnout [ft]
v0 = 656.6  # Initial velocity at burnout [ft/s]
dt = 0.1

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
    return 0.7

# def cd_ADS(mach):
#     # print(6.95556*mach**2 - 0.618889 * mach + 0.3)
#     return 6.95556*mach**2 - 0.618889 * mach + 0.3
#     # return 1

def interpolate_cd_vehicle(Re):
    return 0.7

# def cd_vehicle(density,velocity, height):
#     nu = kinematic_viscosity(height)
#     l = 5.15/144
#     Re = density*velocity*l/nu
#     cd = 0.582 + (0.638 - 0.582) / (3.308e6 - 2.08e6) * (Re - 2.08e6)
#     return cd

### FIX WEIRD LINEAER THING

def ode_solver(t_0, h_0, v_0, dt):
    # Initialize variables
    current_time = t_0
    current_height = h_0
    current_velocity = v_0
    
    # Constants
    # rho_air = density(height_in)  # density of air in slugs/ft^3
    # Cd_vehicle = cd_vehicle(rho_air,current_velocity, current_height)  # Drag coefficient of the vehicle
    # time_step = dt  # time step for Euler integration
    
    states = [[None]] * 5
    # time, height, velocity, mach, drag force
    # times = []
    # heights = []
    # velocities = []
    
    while current_velocity > 0:
        states[0].append(current_time)
        states[1].append(current_height)
        states[2].append(current_velocity)

        T, p, rho, a, nu = fluid_properties(current_height)
        Re = reynolds_number(current_velocity, 5.15/144, nu, rho)
        
        Cd_vehicle = interpolate_cd_vehicle(Re)
        Cd_ads = interpolate_cd_ads(Re)

        F_drag_vehicle = 0.5 * rho * current_velocity**2 * Cd_vehicle * A_vehicle
        F_drag_ADS = 0.5 * rho * current_velocity**2 * Cd_ads * A_ads
        F_gravity = mass * g

        F_net = 0 - F_drag_ADS - F_drag_vehicle - F_gravity

        acceleration_net = F_net / mass

        current_velocity = current_velocity + acceleration_net * dt
        current_height = current_height + current_velocity * dt
        current_time = current_time + dt

    return states


states = ode_solver(0, h0, v0, dt)
plt.plot(states[0], states[1], label=f'ADS Area = {(A_ads)*144} in²')
plt.title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {h0}ft, v0: {v0}ft/s")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (ft)")
plt.legend()
plt.grid(True)
plt.show()

# Constants
# A_vehicle = 24.581150/144  # Reference area of the vehicle [ft^2]
# A_ads = [0, 3/144, 4/144, 5/144, 6/144, 24/144] # ADS areas [ft^2]
# Cd_ads = [1.0, 0.55, 0.55, 0.55, 0.7, 0.97]  # Drag coefficient of ADS (flat plate)


# Simulate for different ADS areas and plot

# for n in range(0,len(A_ads)):
#     times, heights, velocities = compute_apogee(0, h0, v0, mass, A_vehicle, A_ads[n], Cd_ads[n], dt)
#     plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
#     plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=5, ha='left', va='center')


# def mach_helper (height_ft, velocity):
#     #assumptions: same temp as in kinematic_viscosity, air adiabatic index constant, air chemistry constant
#     Tk = (518.67 - 0.003566 * height_ft) * 5/9 #temp in Kelvins
    
#     speed_of_sound = ((1.4*8.314*Tk/0.02896)**0.5)
#     # print(speed_of_sound)
#     mach = (velocity/3.281)/speed_of_sound
#     # print('mach')
#     # print(mach)
#     return mach


# def compute_apogee_mach_func(time_in, height_in, velocity_in, mass, A_vehicle, A_ads, dt):
#     # Initialize variables
#     current_velocity = velocity_in
#     current_height = height_in
#     current_time = time_in
    
#     # Constants
#     g = 32.174  # acceleration due to gravity in ft/s^2
#     rho_air = density(height_in)  # density of air in slugs/ft^3
#     Cd_vehicle = cd_vehicle(rho_air,current_velocity, current_height)  # Drag coefficient of the vehicle
#     time_step = dt  # time step for Euler integration
    
#     times = []
#     heights = []
#     velocities = []
    
#     while current_velocity > 0:
#         times.append(current_time)
#         F_drag_vehicle = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle
#         Cd_ADS = cd_ADS(mach_helper(current_height, current_velocity))
#         F_drag_ADS = 0.5 * rho_air * current_velocity**2 * Cd_ADS * A_ads_mach
#         F_gravity = mass * g
#         F_net = -1 * F_drag_ADS - F_drag_vehicle - F_gravity
#         acceleration_net = F_net / mass
#         current_velocity = current_velocity + acceleration_net * time_step
#         current_height = current_height + current_velocity * time_step
#         current_time += time_step
#         # print(mach_helper(current_height, current_velocity))
        
#         # Save current height and velocity
#         heights.append(current_height)
#         velocities.append(current_velocity)

#     return times, heights, velocities

# # Simulate for different ADS areas and plot
# plt.figure(figsize=(10, 8))


# times, heights, velocities = compute_apogee_mach_func(0, h0, v0, mass, A_vehicle, A_ads, dt)
# plt.plot(times, heights, label=f'ADS Area = {(A_ads_mach)*144} in²')
# plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=10, ha='left', va='center')


# def compute_apogee(time_in, height_in, velocity_in, mass, A_vehicle, A_ads, Cd_ads, dt):
#     # Initialize variables
#     current_velocity = velocity_in
#     current_height = height_in
#     current_time = time_in
    
#     # Constants
#     g = 32.174  # acceleration due to gravity in ft/s^2
#     rho_air = density(height_in)  # density of air in slugs/ft^3
#     Cd_vehicle = cd_vehicle(rho_air,current_velocity, current_height)  # Drag coefficient of the vehicle
#     time_step = dt  # time step for Euler integration
    
#     times = []
#     heights = []
#     velocities = []
    
#     while current_velocity > 0:
#         times.append(current_time)
#         F_drag_vehicle = 0.5 * rho_air * current_velocity**2 * Cd_vehicle * A_vehicle
#         Cd_ADS = cd_ADS(mach_helper(current_height, current_velocity))
#         F_drag_ADS = 0.5 * rho_air * current_velocity**2 * Cd_ADS * A_ads_mach
#         F_gravity = mass * g
#         F_net = -1 * F_drag_ADS - F_drag_vehicle - F_gravity
#         acceleration_net = F_net / mass
#         current_velocity = current_velocity + acceleration_net * time_step
#         current_height = current_height + current_velocity * time_step
#         current_time += time_step
        
#         # Save current height and velocity
#         heights.append(current_height)
#         velocities.append(current_velocity)

#     return times, heights, velocities

# # Simulate for different ADS areas and plot

# for n in range(0,len(A_ads)):
#     times, heights, velocities = compute_apogee(0, h0, v0, mass, A_vehicle, A_ads[n], Cd_ads[n], dt)
#     plt.plot(times, heights, label=f'ADS Area = {round(A_ads[n]*144,2)} in²')
#     plt.text(times[-1], heights[-1], f' {heights[-1]:.1f} ft', fontsize=5, ha='left', va='center')

# # plt.plot(times, mach_nums)
# # Plot settings
# plt.title(f"Rocket Altitude vs Time for Various ADS Area -- h0: {h0}ft, v0: {v0}ft/s")
# plt.xlabel("Time (s)")
# plt.ylabel("Altitude (ft)")
# plt.legend()
# plt.grid(True)
# plt.show()