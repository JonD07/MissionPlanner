import subprocess
import shutil
import random
import io

# Path to the C++ executable
exec_path = "../../build/mission-planner"
exp1_path = "../Experiment1/"


NUM_PLOTS = 50


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
def prepare_inc_drone_scenario(input_file_location, num_drones):
	# Read from scenario.txt
	with open('scenario.txt', 'r') as scenario_file:
		content = scenario_file.read()

	# Add the location of the input
	content += input_file_location+"\n"
	# Add in a random set of drones
	for i in range(num_drones):
		# Pick a random drone to add
		content += random.choice(["0\n15.0 1.0\n", "1\n12.0 1.0\n", "2\n10.0 1.0\n"])

	# Write to scenario_run.txt
	with open('scenario_run.txt', 'w') as scenario_run_file:
		scenario_run_file.write(content)


# Function to run solver and wait for it to finish
def run_executable(alg, results_path, run_num):
	# scenario-file alg plan-flag results-flag results-path run-num
	print(exec_path)
	print('scenario_run.txt')
	print(str(alg))
	print('0')
	print('1')
	print(results_path)
	print(str(run_num))
	process = subprocess.Popen([exec_path, 'scenario_run.txt', str(alg), '0', '1', results_path, str(run_num)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	formatted_stdout = io.BytesIO(stdout).readlines()

	for line in formatted_stdout:
		# print(line.decode())
		if "Found" in line.decode():
			print(line.decode())
	
	
	if stderr:
		print(f"Error:\n{stderr.decode()}")


if __name__ == '__main__':
	# Start with experiment 1 (increasing nodes)
	for n in range(5,201, 5):
		for i in range(NUM_PLOTS):
			prepare_standard_scenario(exp1_path+f"plot_{n}_{i}.txt")
			print(f"Experiment 1 on plot_{n}_{i}.txt")
			# # Run greedy
			# print(f"Running Greedy on plot_{n}_{i}.txt")
			# run_executable(1, exp1_path, i)
			# Run our algorithm
			print(f"Running VRP-CO on plot_{n}_{i}.txt")
			run_executable(3, exp1_path, i)
			# # Run baseline
			# print(f"Running baseline on plot_{n}_{i}.txt")
			# run_executable(4, exp1_path, i)
			# Run VRP only
			print(f"Running VRP only on plot_{n}_{i}.txt")
			run_executable(5, exp1_path, i)
