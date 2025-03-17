#include "Offline_Input.h"

Offline_Input::Offline_Input(std::string scenario_input_path) : Input(scenario_input_path) {
	/*
	  Expected input file structure:
		Location of node data file
		Location of drone data file
		Location of problem input file
		Drone 1 type
		Drone 1 set speed and portion of battery to use
		...
		Drone m type
		Drone m set speed and portion of battery to use

	  Example:
		# Location of node data file
		../data/node_data.dat
		# Location of drone data file
		../data/drone_data.dat
		# Location of problem input file
		../Experiment01/plot_5_0.txt
		# Drone type
		0
		# Set speed: 15, Portion of battery to use: 100%
		15.0 1.0
		# Drone type
		1
		# Set speed: 12, Portion of battery to use: 100%
		12.0 1.0

	 * NOTE: We ignore lines that start with '#'
	 */

	// Did we at least get all of the above?
	if(read_success) {
		// Drone generator
		DroneGenerator droneGenerator(drone_line);

		if(DEBUG_OFFLINEINPUT)
			printf("Reading in drones:\n");

		// We expect an even number of strings for drone data
		if(scenario_data.size()%2 == 0) {
			for(int i = 0; i < boost::numeric_cast<int>(scenario_data.size()); i += 2) {
				// Grab drone type
				int type;
				std::stringstream typeStream(scenario_data.at(i));
				typeStream >> type;

				// Grab drone stats
				double usable_speed, bat_share;
				std::stringstream parameterStream(scenario_data.at(i+1));
				parameterStream >> usable_speed;
				parameterStream >> bat_share;

				// Create a new drone
				Drone* drone = droneGenerator.GenerateDrone(type, usable_speed, bat_share);
				vDroneLst.push_back(drone);

				if(DEBUG_OFFLINEINPUT)
					printf(" %d: %d, v = %f, prct-bat = %f\n", M, type, usable_speed, bat_share);

				M++;
			}
		}
	}

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[Offline_Input::Offline_Input] : Input file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read input\n");
	}
}

Offline_Input::~Offline_Input() {
	// Free memory
	for(Drone* ptr : vDroneLst) {
		delete ptr;
	}
	vDroneLst.clear();
}
