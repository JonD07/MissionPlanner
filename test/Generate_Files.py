import random
import math

INC_NODES = True
INC_ALPHA = True
INC_DATA = True

# Z coordinate range
Z_MIN = -10
Z_MAX = 50
# Data range
Q_MIN = 0.5
Q_MAX = 500.0

# Parameters
NUM_PLOTS = 50

# Increasing nodes
START_COUNT = 5
END_COUNT = 200
Node_Increment = 5
# 25 per km^2
ALPHA = 0.000025

# Increasing alpha
NUM_NODES = 50
START_DENSE = 5
END_DENSE = 151
DENSE_INC = 5

# Increasing Q
START_Q = 0.5
END_Q = 1024
Q_INC_FACTOR = 2


def get_rnd_node():
	node_typs = [0, 1]
	type_i = random.choice(node_typs)
	
	if type_i == 0:
		# Pi 3
		return "0 127.0.0.1"
	else:
		# Pi 4
		return "1 127.0.0.1"
		

# What increases?
if INC_NODES:
	FILE_PATH = "Experiment1/"
	# Loop over the number of sensors to use (n)
	for n in range(START_COUNT, (END_COUNT + Node_Increment), Node_Increment):
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Find the max distance a sensor can from the origin
			MAX_COORD = math.sqrt(n/ALPHA)
			# Open the file
			file_name = f"{FILE_PATH}plot_{n}_{i}.txt"
			with open(file_name, 'w') as file:
				# Number of nodes
				file.write(f"{n}\n")
				for l in range(n):
					# For each node: x y z z_s Q(Mb) type ip-adrs
					# Pick random coordinates
					x = MAX_COORD * random.random() - MAX_COORD/2
					y = MAX_COORD * random.random() - MAX_COORD/2
					z = (Z_MAX - Z_MIN) * random.random() + Z_MIN
					z_s = random.choice([5.0, 10, 15])
					q = (Q_MAX - Q_MIN) * random.random() + Q_MIN
					# Write the results to file
					file.write(f"{x} {y} {z} {z_s} {q} "+get_rnd_node()+"\n")
				# Pick base station
				x_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				y_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				z_b = (Z_MAX - Z_MIN) * random.random() + Z_MIN
				# Record BS position
				file.write(f"0 0 0\n")

if INC_ALPHA:
	FILE_PATH = "Experiment2/"
	# Loop over the number of sensors to use (n)
	for den in range(START_DENSE, (END_DENSE + DENSE_INC), DENSE_INC):
		# Determine alpha
		alpha = den/(1000**2)
		# Find the max distance a sensor can from the origin
		MAX_COORD = math.sqrt(NUM_NODES/alpha)
		print(den, alpha, MAX_COORD)
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{den}_{i}.txt"
			with open(file_name, 'w') as file:
				file.write(f"{NUM_NODES}\n")
				for l in range(NUM_NODES):
					# For each node: x y z z_s Q(Mb) type ip-adrs
					# Pick random coordinates
					x = MAX_COORD * random.random()
					y = MAX_COORD * random.random()
					z = (Z_MAX - Z_MIN) * random.random() + Z_MIN
					z_s = random.choice([5.0, 10, 15])
					q = (Q_MAX - Q_MIN) * random.random() + Q_MIN
					# Write the results to file
					file.write(f"{x} {y} {z} {z_s} {q} "+get_rnd_node()+"\n")
				# Pick base station
				x_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				y_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				z_b = (Z_MAX - Z_MIN) * random.random() + Z_MIN
				# Record BS position
				file.write(f"{x_b} {y_b} {y_b}\n")

if INC_DATA:
	FILE_PATH = "Experiment3/"
	# Gradually increase q
	q = START_Q
	# for q in range(START_Q, (END_Q + DENSE_Q), Q_INC):
	while q <= END_Q:
		# Find the max distance a sensor can from the origin
		MAX_COORD = math.sqrt(NUM_NODES/ALPHA)
		print(q, ALPHA, MAX_COORD)
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{q}_{i}.txt"
			with open(file_name, 'w') as file:
				file.write(f"{NUM_NODES}\n")
				for l in range(NUM_NODES):
					# For each node: x y z z_s Q(Mb) type ip-adrs
					# Pick random coordinates
					x = MAX_COORD * random.random()
					y = MAX_COORD * random.random()
					z = (Z_MAX - Z_MIN) * random.random() + Z_MIN
					z_s = random.choice([5.0, 10, 15])
					# Write the results to file
					file.write(f"{x} {y} {z} {z_s} {q} "+get_rnd_node()+"\n")
				# Pick base station
				x_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				y_b = (MAX_COORD * 2/3) * random.random() + (MAX_COORD * 1/6)
				z_b = (Z_MAX - Z_MIN) * random.random() + Z_MIN
				# Record BS position
				file.write(f"{x_b} {y_b} {y_b}\n")
		q = q * Q_INC_FACTOR
