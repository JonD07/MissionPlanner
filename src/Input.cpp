#include "Input.h"

Input::Input(std::string input_path, int m) {
	// Initial assignment, silence annoying macro warnings
	// and avoid issues if parsing fails
	input_fileName = input_path;
	N = 0;
	M = m;

	/*
	  Expected file structure:
		N
		x_1 y_1 r_1
		....
		x_n y_n r_n
		x_b y_b

	  Example:
		# 5 sensors
		5
		# Sensor data...
		109.9 75.1 21.2
		150.8 172.6 35.4
		58.8 176.8 33.5
		130.7 141.6 44.2
		84.7 161.9 44.6
		# Base station
		31.1 125.4

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
	if(getNextLine(&file, &line)) {
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
		printf(" N = %d, M = %d\n Reading in node data", N, M);

	for(int i = 0; i < N && read_success; i++) {
		// Grab next capability line
		if(getNextLine(&file, &line)) {
			std::stringstream lineStream_i(line);
			double x, y, r;

			lineStream_i >> x;
			lineStream_i >> y;
			lineStream_i >> r;

			if(DEBUG_INPUT)
				printf("  (%f, %f), R: %f\n", x, y, r);
			vNodeLst.push_back(Node(i,x,y,r));
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}

	// Read in the location of the base station
	if(getNextLine(&file, &line)) {
		std::stringstream lineStream_base(line);
		double x, y;

		lineStream_base >> x;
		lineStream_base >> y;

		mBaseStation = Node(-1, x, y, 0);

		if(DEBUG_INPUT)
			printf(" Base station: (%f, %f)\n", x, y);
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
	return 60.0;
}


// Gets the next valid line from file, stores it in line. Will ignore
// lines that start with '#' symbol.
bool Input::getNextLine(std::ifstream* file, std::string* line) {
	bool run_again = true;
	bool read_success = false;
	// Grab next line un-commented line
	while(run_again) {
		if(std::getline(*file, *line)) {
			if(DEBUG_INPUT) {
				printf("Next line: \"%s\"\n", (*line).c_str());
				printf("First char: \'%d\'\n", (*line)[0]);
			}
			// Successfully read next line, verify it doesn't start with '#'
			if((*line)[0] != '#') {
				// Found next valid input line
				run_again = false;
				read_success = true;
			}
			else {
				// Line starts with '#', get next line
				run_again = true;
				if(DEBUG_INPUT)
					puts("Ignoring line");
			}
		}
		else {
			// Failed to read next line
			run_again = false;
			read_success = false;
		}
	}

	if(DEBUG_INPUT)
		printf("Return Line: \"%s\"\n", (*line).c_str());

	return read_success;
}
