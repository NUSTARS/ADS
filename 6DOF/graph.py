import pandas as pd
import matplotlib.pyplot as plt

# Step 1: Load the CSV file
# Replace 'your_file.csv' with the path to your CSV file
file_path = 'data.csv'
data = pd.read_csv(file_path)

# Step 2: Extract the x-axis data
# Replace 'X' with the actual column name for the x-axis
x = data[0]  # X-axis data


plt.plot(data[0],data[5])

# Step 4: Add labels, title, and legend
plt.xlabel('time')  # Replace with your label
plt.ylabel('Angle')  # Replace with your label
plt.title('Graph Title')    # Replace with your title
plt.legend()  # Show legend
plt.grid(True)  # Optional: Add grid for better readability

# Step 5: Show or save the graph
plt.show()
# plt.savefig('output_graph.png')  # Optional: Save the graph as an image

