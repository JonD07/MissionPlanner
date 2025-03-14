import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# Load the CSV file
filename = "pi_3_q_vs_distance.csv"
data = pd.read_csv(filename, delimiter=",")  # Adjust delimiter if necessary

# Transpose the data to ensure each row becomes a separate line
data_other = data.T

# Generate x values using the given formula
num_columns = data.shape[1]
x_values = [(0.001 * 2**(n / 8))*1000 for n in range(num_columns)]

# Create a colormap transitioning from yellow to green to blue
colormap = cm.get_cmap("cool", data_other.shape[0])  # "viridis" smoothly transitions from yellow to blue

# Plot each column as a separate line
plt.figure(figsize=(8, 6))
for index, row in data.iterrows():
	if (index+2) in [2,8,14,20]:
		plt.plot(x_values, row, label=f"{index+2} m/s", color=colormap(index / (data.shape[0] - 1)))
	else:
		plt.plot(x_values, row, color=colormap(index / (data.shape[0] - 1)))

plt.plot([x_values[0],x_values[-1]], [37.622453515,37.622453515], label=f"Expected Max Distance", color='r')

# Labeling and title
plt.xlabel("Data Volume (KB)")
plt.ylabel("Distance from device")
# plt.title("Graph of CSV Data")
plt.legend()
plt.xscale("log")  # Set x-axis to log-base 2
plt.grid()

# Show the plot
plt.show()