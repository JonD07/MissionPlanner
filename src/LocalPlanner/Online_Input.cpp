#include "Online_Input.h"

Online_Input::Online_Input(std::string scenario_input_path) : Input(scenario_input_path) {
	/*
	  Expected input file structure:
		Location of node data file
		Location of drone data file
		Location of problem input file
		Drone type
		Speed, Energy budget
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
		# Speed, Energy budget
		7.5 0.8
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
			printf("Reading in online setup:\n");

		// We expect 5 more lines: drone type, drone speed/budget, location, first node, rest of the nodes
		if(scenario_data.size() == 5) {
			for(int i = 0; i < 5 && read_success; i++) {
				std::stringstream parameterStream(scenario_data.at(i));
				switch(i) {
				case 0:
					// Drone type
					{
						parameterStream >> drone_type;
						M++;
					}
					break;
				case 1:
					// Speed and energy
					{
						double speed, energy_budget;
						parameterStream >> speed;
						parameterStream >> energy_budget;
						vDroneLst.push_back(droneGenerator.GenerateDrone(drone_type, speed, energy_budget));
						M++;
					}
					break;
				case 2:
					// Current location
					{
						parameterStream >> currnt_x;
						parameterStream >> currnt_y;
						parameterStream >> currnt_z;
					}
					break;
				case 3:
					// The first node to visit (over head)
					{
						parameterStream >> first_node;
					}
					break;
				case 4:
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
			printf(" Drone type %d at (%.3f, %.3f, %.3f)\n speed = %0.1f, budget = %0.1f", drone_type, currnt_x, currnt_y, currnt_z,
					vDroneLst.at(0)->GetSpeed(), vDroneLst.at(0)->GetPlannableEnergy());
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
	// Nothing much to see here..
}


// Number of follow-on nodes to visit
int Online_Input::getNk() {
	return boost::numeric_cast<int>(next_list.size());
}


// Get the operational speed of drone l
double Online_Input::getV() {
	if(vDroneLst.size() > 0) {
		return vDroneLst.at(0)->GetSpeed();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Online_Input::getV] : No drone\n");
		exit(1);
	}
}

// Get the time to swap batteries of drone l
double Online_Input::getTb() {
	if(vDroneLst.size() > 0) {
		return vDroneLst.at(0)->GetSwapTime();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Online_Input::getTb] : No drone\n");
		exit(1);
	}
}

// Get rho for moving for drone l (this is in W = J/s)
double Online_Input::getRho_m() {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = set speed
	if(vDroneLst.size() > 0) {
		return vDroneLst.at(0)->GetRhoM();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Online_Input::getRho_m] : No drone\n");
		exit(1);
	}
}

// Get rho for moving for drone l
double Online_Input::getRho_h() {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = 0.... -> y = c4
	if(vDroneLst.size() > 0) {
		return vDroneLst.at(0)->GetRhoH();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Online_Input::getRho_h] : No drone\n");
		exit(1);
	}
}


// Get beta for drone l (planning energy budget, in Jule)
double Online_Input::getB() {
	if(vDroneLst.size() > 0) {
		return vDroneLst.at(0)->GetPlannableEnergy();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Online_Input::getB] : No drone\n");
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


