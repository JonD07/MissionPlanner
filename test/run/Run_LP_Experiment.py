import subprocess
import shutil
import random

# Path to the C++ executable
exec_path = "/home/jonathan/Research/HolisticFramework/MissionPlanner/build/local-planner"
drone_data_path = "/home/jonathan/Research/HolisticFramework/MissionPlanner/test/data/drone_data.dat"
node_data_path = "/home/jonathan/Research/HolisticFramework/MissionPlanner/test/data/node_data.dat"
exp1_path = "/home/jonathan/Research/HolisticFramework/MissionPlanner/test//Experiment2/plot_155_0.txt"

NUM_PLOTS = 20


# Function to read scenario.txt and write to scenario_run.txt
def prepare_standard_scenario(num_nodes):
	# Problem setup
	content = node_data_path+"\n"
	content += drone_data_path+"\n"
	content += exp1_path+"\n"
	# Add in drones
	content += "0\n"
	# Set location (does not really matter)
	content += "0.0 0.0 0.0\n"
	# Node to visit (over-head)
	content += "0\n"
	# Nodes to visit next (ordered list, consider TX rate)
	for n in range(num_nodes):
		content += f"{n} "
		
	# Write to scenario_run.txt
	with open('scenario_run.txt', 'w') as scenario_run_file:
		scenario_run_file.write(content)


# Function to run solver and wait for it to finish
def run_executable(alg, results_path, run_num):
	# scenario-file alg plan-flag results-flag results-path run-num
	process = subprocess.Popen([exec_path, 'scenario_run.txt', '0', '0', '1', results_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	
	if stderr:
		print(f"Error:\n{stderr.decode()}")


if __name__ == '__main__':
	# Start with experiment 1 (increasing nodes)
	for n in range(5, 31, 5):
		for i in range(NUM_PLOTS):
			prepare_standard_scenario(n)
			print(f"N = {n}, iteration {i}")
			run_executable(1, "lp_", i)
