import matplotlib.pyplot as plt
import csv
import numpy as np
import re

# Define a function to strip non-integer values from a string
def strip_non_integer(s):
    return re.sub(r'[^\d.]', '', s)

# Vectorize the function to apply it element-wise to the NumPy array
strip_non_integer_vectorized = np.vectorize(strip_non_integer)

# Initialize an empty list to store the CSV data
raw_data = []

# Open the CSV file
with open('FT5_primary.csv', 'r') as csvfile:
    # Create a CSV reader object
    csvReader = csv.reader(csvfile)
    next(csvReader)

    # Loop through each row in the CSV file
    for row in csvReader:
        # Append the row to the list
        raw_data.append(row)


array = np.array(raw_data)
time = strip_non_integer_vectorized(array[:,4]).astype(float)
height = strip_non_integer_vectorized(array[:,10]).astype(float)
speed = strip_non_integer_vectorized(array[:,11]).astype(float)

i = np.where(height == np.max(height))[0][0]

print(i)

fig, ax = plt.subplots()
ax.plot(time[0:i], speed[0:i], time[0:i], height[0:i], linewidth=2.0)

plt.show()