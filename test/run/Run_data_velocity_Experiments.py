# #################################
#  Run this file in the /test/run/ directory



import subprocess
import shutil
import random
import io, re, ast, math
import csv

# Path to the C++ executable

exec_path = "../../build/mission-planner"
exp1_path = "../data_velocity_test/"
node_path = "../data/node_data.dat"
drone_path = "../data/drone_data.dat"


NUM_PLOTS = 50

hovering_points_table = []


def write_to_csv(filename, data):
    """
    Writes a 2D lookup table (list of lists) into a CSV file.

    :param filename: Name of the CSV file.
    :param data: 2D list representing the lookup table.
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)


# Function to read scenario.txt and write to scenario_run.txt
def prepare_standard_scenario(input_file_location):
	# Read from scenario.txt
	with open('scenario.txt', 'r') as scenario_file:
		content = scenario_file.read()

	# Add the location of the input
	content += input_file_location+"\n"
	# Add in drones
	content += "0\n15.0 1.0\n1\n12.0 1.0\n2\n10.0 1.0"

	# Write to scenario_run.txt
	with open('scenario_run.txt', 'w') as scenario_run_file:
		scenario_run_file.write(content)


# Function to read scenario.txt and write to scenario_run.txt
def prepare_data_scenario(input_file_location, velocity):
	# Read from scenario.txt
	with open('scenario_run.txt', 'r') as scenario_file:
		content = scenario_file.readlines()

	# Add the location of the input
	content[5] = input_file_location+"\n"

	# Set velocity and battery proportion
	content[7] = str(velocity) + " 1.0" + "\n"
	# What does this do?
	# # Add in a random set of drones
	# for i in range(num_drones):
	# 	# Pick a random drone to add
	# 	content += random.choice(["0\n15.0 1.0\n", "1\n12.0 1.0\n", "2\n10.0 1.0\n"])

	# Write to scenario_run.txt
	with open('scenario_run.txt', 'w') as scenario_run_file:
		scenario_run_file.writelines(content)


# Function to run solver and wait for it to finish
def run_executable(alg, results_path, run_num):
	# scenario-file alg plan-flag results-flag results-path run-num
	process = subprocess.Popen([exec_path, 'scenario_run.txt', str(alg), '0', '1', results_path, str(run_num)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	formatted_stdout = io.BytesIO(stdout).readlines()

	print(formatted_stdout[-4].decode())

	match = re.search(r"\(([^)]+)\)", formatted_stdout[-4].decode())  # Find the tuple-like part

	# for line in formatted_stdout:
	# 	# print(line.decode())
	# 	if "!!!" in line.decode():
	# 		print(line.decode())

	if match:
		tuple_test = f"({match.group(1)})"  # Reconstruct the tuple string
		first_element = ast.literal_eval(tuple_test)[0]  # Safely evaluate and extract the first element
		second_element = ast.literal_eval(tuple_test)[1]
		third_element = ast.literal_eval(tuple_test)[2]
		print(first_element)
		print(second_element)
		print(third_element)
		distance_from_sensor = math.sqrt((100 - float(first_element))**2 + (100 - float(second_element))**2 + (10 - float(third_element))**2)
		print(distance_from_sensor)

		hovering_points_list.append(distance_from_sensor)
	# tuple_test = eval(formatted_stdout[-2].decode())
	# print(tuple_test)
	# print(formatted_stdout[-2].decode())
	if stderr:
		print(f"Error:\n{stderr.decode()}")


if __name__ == '__main__':

# Run data_velocity experiment (increasing data)
	q = 0
	# for pi in range(2):
	for speed in range(2,21):
		hovering_points_list = []
		for i in range(160):
			prepare_data_scenario(exp1_path + f"plot_{i}.txt", speed)
			print(f"Data Velocity Experiment on plot_{i}.txt")
			# Run our algorithm
			print(f"Running VRP-CO on plot_{i}.txt")
			run_executable(3, exp1_path, q)
		hovering_points_table.append(hovering_points_list)
		
	output_data_vel_path = exp1_path + "data_velocity_output_.txt"

	write_to_csv(output_data_vel_path, hovering_points_table)
	# with open(output_data_vel_path, 'w') as file:
	# 	for item in hovering_points_list:
	# 		file.write(f"{item}\n")
