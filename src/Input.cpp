#include "Input.h"

Input::Input(std::string scenario_input_path) : input_fileName(scenario_input_path) {
	// Initial assignment, silence annoying macro warnings
	// and avoid issues if parsing fails
	N = 0;
	M = 0;

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

	if(SANITY_PRINT)
		printf("Reading input\n");

	// Read status
	bool read_success = true;

	// File reader, for handling scenario file
	FileReader scenarioFileReader(scenario_input_path);

	// Get node data file
	std::string node_line;
	if(!scenarioFileReader.GetNextLine(&node_line)) {
		// Line reading failed
		read_success = false;
	}

	// Get drone data file
	std::string drone_line;
	if(read_success && !scenarioFileReader.GetNextLine(&drone_line)) {
		// Line reading failed
		read_success = false;
	}

	// Get problem input file
	std::string input_line;
	if(read_success && !scenarioFileReader.GetNextLine(&input_line)) {
		// Line reading failed
		read_success = false;
	}

	// Did we at least get all of the above?
	if(read_success) {
		// Node generator
		NodeGenerator nodeGenerator(node_line);
		// Drone generator
		DroneGenerator droneGenerator(drone_line);

		if(DEBUG_INPUT)
			printf("Reading in drones:\n");

		// Finish reading scenario file
		std::string type_line;
		while(scenarioFileReader.GetNextLine(&type_line) && read_success) {
			// Grab drone type
			int type;
			std::stringstream typeStream(type_line);
			typeStream >> type;

			// Grab drone stats
			double usable_speed, bat_share;
			std::string parameter_line;
			if(scenarioFileReader.GetNextLine(&parameter_line)) {
				std::stringstream parameterStream(parameter_line);
				parameterStream >> usable_speed;
				parameterStream >> bat_share;

				// Create a new drone
				Drone* drone = droneGenerator.GenerateDrone(type, usable_speed, bat_share);
				vDroneLst.push_back(drone);

				if(DEBUG_INPUT)
					printf(" %d: %d, v = %f, prct-bat = %f\n", M, type, usable_speed, bat_share);

				M++;
			}
			else {
				// Line reading failed
				read_success = false;
			}
		}

		/*
		  Expected input file structure:
			N
			x_1 y_1 z_1 zs_1 q_1 t_1
			....
			x_n y_n z_n zs_n q_n t_n
			x_b y_b z_b

		  Example:
			# 5 sensors
			5
			# Sensor data...
			109.9 75.1  21.1 1.5 25.0 1
			150.8 172.6 35.3 0.8 40.0 1
			58.8  176.8 3.5  1.0 10.0 0
			130.7 141.6 34.2 0.5 40.0 0
			84.7  161.9 4.5  2.0 10.0 0
			# Base station
			31.1  125.4 2.0

		 * NOTE: We ignore lines that start with '#'
		 */

		// Create file reader for problem input file
		FileReader inputFileReader(input_line);

		// Open file
		std::string line;
		// Grab first line, should have N
		if(inputFileReader.GetNextLine(&line)) {
			std::stringstream lineStreamNEM(line);
			// Push value into N
			lineStreamNEM >> N;
		}
		else {
			// Line reading failed
			read_success = false;
		}

		// Sanity print
		if(SANITY_PRINT)
			printf(" N = %d, M = %d\n Reading in node data\n", N, M);

		for(int i = 0; i < N && read_success; i++) {
			// Grab next capability line
			if(inputFileReader.GetNextLine(&line)) {
				// Create a new node
				Node* node = nodeGenerator.GenerateNode(i,line);
				vNodeLst.push_back(node);
				if(DEBUG_INPUT)
					printf("  %d: (%f, %f, %f) safe-alt: %f, data: %f\n", vNodeLst.back()->getID(),
							vNodeLst.back()->getX(), vNodeLst.back()->getY(), vNodeLst.back()->getZ(),
							vNodeLst.back()->getZs(), vNodeLst.back()->getQ());
			}
			else {
				// Line reading failed
				read_success = false;
			}
		}

		// Read in the location of the base station
		if(inputFileReader.GetNextLine(&line)) {
			std::stringstream lineStream_base(line);
			double x, y, z;

			lineStream_base >> x;
			lineStream_base >> y;
			lineStream_base >> z;

			mBaseStation = BaseStation(x, y, z);

			if(DEBUG_INPUT)
				printf(" Base station: (%f, %f, %f)\n", mBaseStation.fX, mBaseStation.fY, mBaseStation.fZ);
		}
		else {
			// Line reading failed
			read_success = false;
		}

	}

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[Input::Input] : Input file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read input\n\n");
	}
}

Input::~Input() {
	// Free memory
	for(Node* ptr : vNodeLst) {
		delete ptr;
	}
	vNodeLst.clear();
}


// Get the operational speed of drone l
double Input::getV_l(int l) {
	if(l >= 0 && l < boost::numeric_cast<int>(vDroneLst.size())) {
		return vDroneLst.at(l)->GetSpeed();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getV_l] : Bad drone index\n");
		exit(1);
	}
}

// Get the time to swap batteries of drone l
double Input::getTb_l(int l) {
	if(l >= 0 && l < boost::numeric_cast<int>(vDroneLst.size())) {
		return vDroneLst.at(l)->GetSwapTime();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getV_l] : Bad drone index\n");
		exit(1);
	}
}

// Get rho for moving for drone l (this is in W = J/s)
double Input::getRho_m(int l) {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = set speed
	if(l >= 0 && l < boost::numeric_cast<int>(vDroneLst.size())) {
		return vDroneLst.at(l)->GetRhoM();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : Bad drone index\n");
		exit(1);
	}
}

// Get rho for moving for drone l
double Input::getRho_h(int l) {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	//   where x = 0.... -> y = c4
	if(l >= 0 && l < boost::numeric_cast<int>(vDroneLst.size())) {
		return vDroneLst.at(l)->GetRhoH();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : Bad drone index\n");
		exit(1);
	}
}


// Get beta for drone l (planning energy budget, in Jule)
double Input::getB_l(int l) {
	if(l >= 0 && l < boost::numeric_cast<int>(vDroneLst.size())) {
		return vDroneLst.at(l)->GetPlannableEnergy();
	}
	else {
		// Shouldn't be asking for something that does not exist..
		fprintf(stderr, "[Input::getRho_m] : Bad drone index\n");
		exit(1);
	}
}

// Get the x-coordinate of node i
double Input::getX_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getX_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getX();
}

// Get the y-coordinate of node i
double Input::getY_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getY();
}

// Get the z-coordinate of node i
double Input::getZ_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getZ();
}

// Get node i's safe altitude
double Input::getZs_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getZs();
}

// Get node i's data quantity to collect
double Input::getQ_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getQ();
}

// Get data TX parameters for node i
void Input::getTXParams_i(int i, double* a, double* b, double* mrate) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	vNodeLst.at(i)->getTXParams(a, b, mrate);
}

// Get the "agnostic" max TX range for node i
double Input::getR_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i)->getR();
}


