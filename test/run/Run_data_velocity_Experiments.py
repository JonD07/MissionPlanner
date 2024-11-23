import subprocess
import shutil
import random
import io, re, ast, math

# Path to the C++ executable

exec_path = "../../build/mission-planner"
exp1_path = "../data_velocity_test/"


NUM_PLOTS = 50

hovering_points_list = []


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
def prepare_data_scenario(input_file_location, num_drones):
	# Read from scenario.txt
	with open('scenario_run.txt', 'r') as scenario_file:
		content = scenario_file.readlines()

	# Add the location of the input
	content[5] = input_file_location+"\n"

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

	match = re.search(r"\(([^)]+)\)", formatted_stdout[-2].decode())  # Find the tuple-like part

	if match:
		tuple_test = f"({match.group(1)})"  # Reconstruct the tuple string
		first_element = ast.literal_eval(tuple_test)[0]  # Safely evaluate and extract the first element
		second_element = ast.literal_eval(tuple_test)[1]
		third_element = ast.literal_eval(tuple_test)[2]
		print(first_element)
		print(second_element)
		print(third_element)
	distance_from_sensor = math.sqrt((100 - float(first_element))**2 + float(second_element)**2 + float(third_element)**2)
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
	for i in range(40):
		prepare_data_scenario(exp1_path + f"plot_{i}.txt",1)
		print(f"Data Velocity Experiment on plot_{i}.txt")
		# Run greedy
		# Run our algorithm
		print(f"Running VRP-CO on plot_{i}.txt")
		run_executable(3, exp1_path, q)
		
	output_data_vel_path = "data_velocity_output.txt"
	with open(output_data_vel_path, 'w') as file:
		for item in hovering_points_list:
			file.write(f"{item}\n")
