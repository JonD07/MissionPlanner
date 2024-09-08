#include "Input.h"

Input::Input(std::string input_path, std::string data_path, int m) : input_fileName(input_path), fileReader(input_path), nodeGenerator(data_path) {
	// Initial assignment, silence annoying macro warnings
	// and avoid issues if parsing fails
	N = 0;
	M = m;

	/*
	  Expected file structure:
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

	if(SANITY_PRINT)
		printf("Reading input\n");

	// Read status
	bool read_success = true;

	// Open file
	std::ifstream file(input_path);
	std::string line;
	// Grab first line, should have N, M, and  R
	if(fileReader.GetNextLine(&line)) {
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
		if(fileReader.GetNextLine(&line)) {
			// Create a new node
			Node node = nodeGenerator.GenerateNode(i,line);
			vNodeLst.push_back(node);
			if(DEBUG_INPUT)
				printf("  %d: (%f, %f, %f) safe-alt: %f, data: %f\n", vNodeLst.back().getID(),
						vNodeLst.back().getX(), vNodeLst.back().getY(), vNodeLst.back().getZ(),
						vNodeLst.back().getZs(), vNodeLst.back().getQ());
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}

	// Read in the location of the base station
	if(fileReader.GetNextLine(&line)) {
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
}


// Get the operational speed of drone l
// TODO: Don't just return a constant!!
double Input::getV_l(int l) {
	return 12.0;
}

// Get the time to swap batteries of drone l
// TODO: Don't just return a constant!!
double Input::getTb_l(int l) {
	return 120.0;
}

// Get the x-coordinate of node i
double Input::getX_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getX_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getX();
}

// Get the y-coordinate of node i
double Input::getY_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getY();
}

// Get the z-coordinate of node i
double Input::getZ_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getZ();
}

// Get node i's safe altitude
double Input::getZs_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getZs();
}

// Get node i's data quantity to collect
double Input::getQ_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getQ();
}

// Get data TX parameters for node i
void Input::getTXParams_i(int i, double* a, double* b, double* mrate) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	vNodeLst.at(i).getTXParams(a, b, mrate);
}

// Get the "agnostic" max TX range for node i
double Input::getR_i(int i) {
	// Range check..
	if(i < 0 || i >= N) {
		fprintf(stderr,"[ERROR:Input::getY_i] Bad index : %d\n", i);
		exit(1);
	}

	return vNodeLst.at(i).getR();
}


