import pandas as pd
import matplotlib.pyplot as plt

# Step 1: Load the CSV file
# Replace 'your_file.csv' with the path to your CSV file
file_path = 'data.csv'

names = ["time", "altitude", "vx", "vy", "vz", "wx", "wy", "wz", "thetax", "thetay", "thetaz"]
data = pd.DataFrame(columns=names)


# Read data from CSV
csv_data = pd.read_csv(file_path)

if list(csv_data.columns) != names:
    csv_data.columns = names  # Assign names if CSV does not have headers or has mismatched headers

data = pd.concat([data, csv_data], ignore_index=True)

# print(data)

# Step 2: Extract the x-axis data
# Replace 'X' with the actual column name for the x-axis
x = data["time"]  # X-axis data




plt.plot(x,data["thetax"])

# Step 4: Add labels, title, and legend
plt.xlabel('time')  # Replace with your label
plt.ylabel('Angle')  # Replace with your label
plt.title('Graph Title')    # Replace with your title
plt.legend()  # Show legend
plt.grid(True)  # Optional: Add grid for better readability

# Step 5: Show or save the graph
plt.show()
# plt.savefig('output_graph.png')  # Optional: Save the graph as an image

