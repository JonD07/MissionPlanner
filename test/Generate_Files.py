import random
import math
from MAVProxy.modules.mavproxy_map import srtm

INC_NODES = False
INC_ALPHA = False
INC_DATA = False
FW_TEST = True
DUMMY = False

# Z coordinate range
Z_MIN = -10
Z_MAX = 50
# Data range
Q_MIN = 0.125
Q_MAX = 64.0

# Parameters
NUM_PLOTS = 1

# Increasing nodes
START_COUNT = 5
END_COUNT = 200
Node_Increment = 5
# 75 per km^2
ALPHA = 0.000075
# 75 per km^2
FW_ALPHA = 0.000015

# Increasing alpha
NUM_NODES = 50
START_DENSE = 5
END_DENSE = 151
DENSE_INC = 5

# Increasing Q
START_Q = 0.001
END_Q = 524.289
Q_INC_FACTOR = 2

# Base location
BASE_GPS = (39.739550, -105.222467)  # Latitude, Longitude
BASE_XYZ = (0, 0, 0)  # x, y, z relative coordinates

# Other constants
r_earth = 6378000


def get_lat_lon(relative_x, relative_y):
	new_latitude = BASE_GPS[0] + (relative_y / r_earth) * (180 / math.pi)
	new_longitude = BASE_GPS[1] + (relative_x / r_earth) * (180 / math.pi) / math.cos(BASE_GPS[0] * math.pi/180)
	return (new_latitude, new_longitude)


def get_altitude(x, y):
	# Get the lat,long of this position
	lat_long = get_lat_lon(x, y)
	# Get the SRTM tile
	downloader = srtm.SRTMDownloader()
	downloader.loadFileList()
	tile = downloader.getTile(int(math.floor(lat_long[0])), int(math.floor(lat_long[1])))
	# Return altitude from the tile
	return tile.getAltitudeFromLatLon(lat_long[0], lat_long[1])


def get_relative_z(x, y):
	# Get the altitude of the base
	base_alt = get_altitude(BASE_XYZ[0], BASE_XYZ[1])
	# Get the altitude at this x,y position
	point_alt = get_altitude(x, y)
	# Return differance (plus base)
	return point_alt - (base_alt + BASE_XYZ[2])


def get_rnd_node():
	node_typs = [0, 1]
	type_i = random.choice(node_typs)
	
	if type_i == 0:
		# Pi 3
		return "0 127.0.0.1"
	else:
		# Pi 4
		return "1 127.0.0.1"
		

def get_rnd_q():
	factor = random.choice(list(range(0, 16)))
	return START_Q*2**factor


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
	FILE_PATH = "Experiment4/"
	# Gradually increase q
	q = START_Q
	step_count = 0
	# for q in range(START_Q, (END_Q + DENSE_Q), Q_INC):
	while q <= END_Q:
		# Find the max distance a sensor can from the origin
		MAX_COORD = math.sqrt(NUM_NODES/ALPHA)
		print(q, ALPHA, MAX_COORD)
		# Generate NUM_PLOTS plots
		for i in range(NUM_PLOTS):
			# Open the file
			file_name = f"{FILE_PATH}plot_{step_count}_{i}.txt"
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
		step_count += 1

if FW_TEST:
	FILE_PATH = "FW_Test/"
	# Loop over the number of sensors to use (n)
	for n in range(5, 35, 5):
		# Generate NUM_PLOTS plots
		for i in range(20):
			# Find the max distance a sensor can from the origin
			MAX_COORD = math.sqrt(n/FW_ALPHA)
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
					z = get_relative_z(x, y)
					z_s = random.choice(list(range(8, 20, 2)))
					q = get_rnd_q()
					# Write the results to file
					file.write(f"{x} {y} {z} {z_s} {q} "+get_rnd_node()+"\n")
				# Record BS position
				file.write(f"{BASE_XYZ[0]} {BASE_XYZ[1]} {BASE_XYZ[2]}\n")

if DUMMY:
	# Open the file
	with open("FW_Test/dummy_.txt", 'w') as file:
		# Number of nodes
		file.write(f"4\n")
		for l in range(4):
			# For each node: x y z z_s Q(Mb) type ip-adrs
			# Pick random coordinates
			if l == 0:
				x = 300
				y = 300
			elif l == 1:
				x = 300
				y = -300
			elif l == 2:
				x = -300
				y = -300
			elif l == 3:
				x = -300
				y = 300
			z = get_relative_z(x, y)
			z_s = 8
			q = 32.0
			# Write the results to file
			file.write(f"{x} {y} {z} {z_s} {q} " + get_rnd_node() + "\n")
		# Record BS position
		file.write(f"{BASE_XYZ[0]} {BASE_XYZ[1]} {BASE_XYZ[2]}\n")

