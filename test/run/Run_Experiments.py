import subprocess
import shutil
import random
import io

# Path to the C++ executable
ORCHESTRATOR = "/home/jonathan/Research/HolisticFramework/"
exec_path = ORCHESTRATOR+"MissionPlanner/build/mission-planner"
exp1_path = ORCHESTRATOR+"MissionPlanner/test/Experiment1/"
exp2_path = ORCHESTRATOR+"MissionPlanner/test/Experiment2/"
exp3_path = ORCHESTRATOR+"MissionPlanner/test/Experiment3/"
exp4_path = ORCHESTRATOR+"MissionPlanner/test/Experiment4/"

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
def prepare_twodrones_scenario(input_file_location):
	# Read from scenario.txt
	with open('scenario.txt', 'r') as scenario_file:
		content = scenario_file.read()

	# Add the location of the input
	content += input_file_location+"\n"
	# Add in drones
	content += "1\n12.0 1.0\n2\n10.0 1.0"

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
	process = subprocess.Popen([exec_path, 'scenario_run.txt', str(alg), '0', '1', results_path, str(run_num)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	if stderr:
		print(f"Error:\n{stderr.decode()}")


if __name__ == '__main__':
	# Start with experiment 1 (increasing nodes)
	for n in range(5,201, 5):
		for i in range(NUM_PLOTS):
			prepare_standard_scenario(exp1_path+f"plot_{n}_{i}.txt")
			# Run our algorithm
			print(f"Experiment 1: Running VRP-CO on plot_{n}_{i}.txt")
			run_executable(3, exp1_path, i)
			# Run baseline
			print(f"Experiment 1: Running baseline on plot_{n}_{i}.txt")
			run_executable(4, exp1_path, i)
			# Run VRP only
			print(f"Experiment 1: Running VRP only on plot_{n}_{i}.txt")
			run_executable(5, exp1_path, i)
	
	# Run experiment 2 (increasing density)
	for n in range(5, 151, 5):
		for i in range(NUM_PLOTS):
			prepare_standard_scenario(exp2_path + f"plot_{n}_{i}.txt")
			# Run our algorithm
			print(f"Experiment 2: Running VRP-CO on plot_{n}_{i}.txt")
			run_executable(3, exp2_path, n)
			# Run baseline
			print(f"Experiment 2: Running baseline on plot_{n}_{i}.txt")
			run_executable(4, exp2_path, n)
			# Run VRP only
			print(f"Experiment 2: Running VRP only on plot_{n}_{i}.txt")
			run_executable(5, exp2_path, n)
	
	# Run experiment 1.5 (increasing drones)
	n = 75
	for m in range(1, 11):
		for i in range(NUM_PLOTS):
			prepare_inc_drone_scenario(exp1_path + f"plot_{n}_{i}.txt", m)
			# Run our algorithm
			print(f"Experiment 1.5: Running VRP-CO on plot_{n}_{i}.txt")
			run_executable(3, exp1_path, i)
			# Run baseline
			print(f"Experiment 1.5: Running baseline on plot_{n}_{i}.txt")
			run_executable(4, exp1_path, i)
			# Run VRP only
			print(f"Experiment 1.5: Running VRP only on plot_{n}_{i}.txt")
			run_executable(5, exp1_path, i)

	# Run experiment 3 (increasing data)
	step_count = 0
	while step_count <= 16:
		for i in range(NUM_PLOTS):
			prepare_standard_scenario(exp3_path + f"plot_{step_count}_{i}.txt")
			# Run our algorithm
			print(f"Experiment 3: Running VRP-CO on plot_{step_count}_{i}.txt")
			run_executable(3, exp3_path, step_count)
			# Run baseline
			print(f"Experiment 3: Running baseline on plot_{step_count}_{i}.txt")
			run_executable(4, exp3_path, step_count)
			# Run VRP only
			print(f"Experiment 3: Running VRP only on plot_{step_count}_{i}.txt")
			run_executable(5, exp3_path, step_count)
		step_count += 1

	# Experiment 4 (small increasing nodes)
	for n in range(5, 31, 1):
		for i in range(NUM_PLOTS):
			prepare_twodrones_scenario(exp4_path+f"plot_{n}_{i}.txt")
			# Run our algorithm
			print(f"Experiment 4: Running VRP-CO on plot_{step_count}_{i}.txt")
			run_executable(3, exp4_path, step_count)
			# Run baseline
			print(f"Experiment 4: Running baseline on plot_{step_count}_{i}.txt")
			run_executable(4, exp4_path, step_count)
			# Run VRP only
			print(f"Experiment 4: Running VRP only on plot_{step_count}_{i}.txt")
			run_executable(5, exp4_path, step_count)

	# Experiment 4 (small increasing nodes) with optimal solver..
	for n in range(5, 31, 1):
		for i in range(NUM_PLOTS):
			prepare_twodrones_scenario(exp4_path+f"plot_{n}_{i}.txt")
			# Optimal solver
			print(f"Experiment 4: Running Optimal Solver on plot_{n}_{i}.txt")
			run_executable(0, exp4_path, i)

