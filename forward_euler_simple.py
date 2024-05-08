import matplotlib.pyplot as plt
import csv
import numpy as np
import re

# Define a function to strip non-integer values from a string
def strip_non_integer(s):
    return re.sub(r'[^\d.-]', '', s)

def load_data(filename):
    # Vectorize the function to apply it element-wise to the NumPy array
    strip_non_integer_vectorized = np.vectorize(strip_non_integer)

    # Initialize an empty list to store the CSV data
    raw_data = []

    # Open the CSV file
    with open(filename, 'r') as csvfile:
        # Create a CSV reader object
        csvReader = csv.reader(csvfile)
        next(csvReader)

        # Loop through each row in the CSV file
        for row in csvReader:
            # Append the row to the list
            raw_data.append(row)

    array = np.array(raw_data)
    time = strip_non_integer_vectorized(array[:, 4]).astype(float)
    height = strip_non_integer_vectorized(array[:, 10]).astype(float)
    speed = strip_non_integer_vectorized(array[:, 11]).astype(float)
    accel = strip_non_integer_vectorized(array[:, 7]).astype(float)

    i = np.where(array[:, 6] == '   coast')[0][0] # start of coast
    j = np.where(height == np.max(height))[0][0] # apogee

    return time[i:j] - time[i], 3.28*height[i:j], 3.28*speed[i:j], 3.28*accel[i:j]


def compute_apogee(height_in, velocity_in, acceleration_in, mass, area, dt):
    # Constants
    g = 32.174  # acceleration due to gravity in ft/s^2
    rho_air = 0.002378  # density of air in slugs/ft^3
    Cd = 0.3 # drag coefficient
    
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


#----------------Loading Input Data-------------------------------------------------

time, height, speed, accel = load_data('FT5_primary.csv')

#----------------Running Euler----------------------------------------------

acceleration = -32.17  # Constant acceleration (assuming downwards as negative) in ft/s^2
mass = 37.15/32.17  # Mass of the rocket in slugs (1 slug = 32.174 lbs·s^2/ft)
area = 0.25  # Cross-sectional area of the rocket in ft^2

# apogee = compute_apogee(current_height, velocity, acceleration, mass, area, 0.01)
# print("Apogee reached:", apogee, "feet")

dt = 0.01

guesses = []

current_height = height[0]  # Initial height in feet
velocity = speed[0]  # Initial velocity in ft/s
acceleration = accel[0]
initialTime, initialHeight, initialSpeed = compute_apogee(current_height, velocity, acceleration, mass, area, dt)

for i in range(0,len(time)):
    current_height = height[i]  # Initial height in feet
    velocity = speed[i]  # Initial velocity in ft/s
    acceleration = accel[i]
    eulerTime, eulerHeight, eulerSpeed = compute_apogee(current_height, velocity, acceleration, mass, area, dt)
    if eulerHeight:
        maxHeight = np.max(eulerHeight)
        guesses.append(maxHeight)

plt.plot(initialTime, initialHeight, time, height)
#plt.plot(time[0:len(guesses)], guesses - max(height))
plt.legend(["Initial Euler Prediction", "Real"])
plt.xlabel("time (s)")
plt.ylabel("height (ft)")
plt.title("Error of predicted apogee over time")
plt.show()
