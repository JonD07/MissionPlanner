#include "Online_Input.h"

Online_Input::Online_Input(std::string scenario_input_path) : Input(scenario_input_path) {
	/*
	  Expected input file structure:
		Location of node data file
		Location of drone data file
		Location of problem input file
		Drone type
		Current location
		Node # to visit (over-head)
		Nodes to visit next (ordered list, consider TX rate)

	  Example:
		# Location of node data file
		../data/node_data.dat
		# Location of drone data file
		../data/drone_data.dat
		# Location of problem input file
		../Experiment01/plot_5_0.txt
		# Drone type
		0
		# Current location
		82.19 144.48 19.31
		# Node to visit (over-head)
		4
		# Nodes to visit next (ordered list, consider TX rate)
		3 2 1 0

	 * NOTE: We ignore lines that start with '#'
	 */

	// Did we at least get all of the above?
	if(read_success) {
		// Drone generator
		DroneGenerator droneGenerator(drone_line);

		if(DEBUG_ONLINEINPUT)
			printf("Reading in setup:\n");

		// We expect 4 more lines: drone type, location, first node, rest of the nodes
		if(scenario_data.size() == 4) {
			for(int i = 0; i < 4 && read_success; i++) {
				std::stringstream parameterStream(scenario_data.at(i));
				switch(i) {
				case 0:
					// Drone type
					{
						parameterStream >> drone_type;
						drone = droneGenerator.GenerateDrone(drone_type);
						M++;
					}
					break;
				case 1:
					// Current location
					{
						parameterStream >> currnt_x;
						parameterStream >> currnt_y;
						parameterStream >> currnt_z;
					}
					break;
				case 2:
					// The first node to visit (over head)
					{
						parameterStream >> first_node;
					}
					break;
				case 3:
					// List of next nodes
					{
						int next_node;
						while(static_cast<bool>(parameterStream >> next_node)) {
							next_list.push_back(next_node);
						}
					}
					break;
				}
			}
		}
		else {
			read_success = false;
		}

		if(DEBUG_ONLINEINPUT) {
			printf(" Drone type %d at (%.3f, %.3f, %.3f)\n", drone_type, currnt_x, currnt_y, currnt_z);
			printf(" Nodes to visit:\n  %d (overhead)", first_node);
			for(int n : next_list) {
				printf(" %d", n);
			}
			puts("");
		}
	}


	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[Online_Input::Online_Input] : Input file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read input\n");
	}
}

Online_Input::~Online_Input() {
	delete drone;
}


// Number of follow-on nodes to visit
int Online_Input::getNk() {
	return boost::numeric_cast<int>(next_list.size());
}


// Get the operational speed of drone l
double Online_Input::getV() {
	if(drone != NULL) {
		return drone->GetSpeed();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getV] : No drone\n");
		exit(1);
	}
}

// Get the time to swap batteries of drone l
double Online_Input::getTb() {
	if(drone != NULL) {
		return drone->GetSwapTime();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getV_l] : No drone\n");
		exit(1);
	}
}

// Get rho for moving for drone l (this is in W = J/s)
double Online_Input::getRho_m() {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = set speed
	if(drone != NULL) {
		return drone->GetRhoM();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : No drone\n");
		exit(1);
	}
}

// Get rho for moving for drone l
double Online_Input::getRho_h() {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = 0.... -> y = c4
	if(drone != NULL) {
		return drone->GetRhoH();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : No drone\n");
		exit(1);
	}
}


// Get beta for drone l (planning energy budget, in Jule)
double Online_Input::getB() {
	if(drone != NULL) {
		return drone->GetPlannableEnergy();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : No drone\n");
		exit(1);
	}
}

// Get the x-coordinate of the first node to visit
double Online_Input::getX_f() {
	return vNodeLst.at(first_node)->getX();
}

// Get the y-coordinate of the first node to visit
double Online_Input::getY_f() {
	return vNodeLst.at(first_node)->getY();
}

// Get the z-coordinate of the first node to visit
double Online_Input::getZ_f() {
	return vNodeLst.at(first_node)->getZ();
}


