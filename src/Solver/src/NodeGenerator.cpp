#include "NodeGenerator.h"

NodeGenerator::NodeGenerator(std::string data_path) : data_fileName(data_path), fileReader(data_path) {
	/*
	  Expected data file structure:
		ID_1
		A_1 B_1 maxRate_1 C_1 R_1
		...
		ID_n
		A_n B_n maxRate_n C_n R_n

	  Example:
		# Pi 3
		0
		100.0 0.01 0.2 0.077 50
		# Pi 4
		1
		200.0 0.01 0.2 0.077 70

	 * NOTE: We ignore lines that start with '#'
	 */

	if(DEBUG_NODE_GEN)
		printf("Reading Node Data\n");

	// Read status
	bool read_success = true;

	std::string id_line;
	// Grab next line
	while(fileReader.GetNextLine(&id_line) && read_success) {
		// Grab the node id
		int id;
		std::stringstream idStream(id_line);
		idStream >> id;

		// Grab node parameters
		double a, b, maxR, C, r, minR;
		std::string parameter_line;
		if(fileReader.GetNextLine(&parameter_line)) {
			std::stringstream parameterStream(parameter_line);
			parameterStream >> a;
			parameterStream >> b;
			parameterStream >> maxR;
			parameterStream >> C;
			parameterStream >> r;
			parameterStream >> minR;

			// Create a new node parameter object
			NodeParameters parameters(id, a, b, maxR, C, r, minR);
			nodeParameters.insert(std::pair<int, NodeParameters>(id, parameters));
		}
		else {
			// Line reading failed
			read_success = false;
		}
	}

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[NodeGenerator::NodeGenerator] : Node data file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read node data\n");
	}
}

NodeGenerator::~NodeGenerator() {}

// Takes in a node string and returns a node. The string is expected to be in this format:
//  x_1 y_1 z_1 zs_1 q_1 t_1
Node* NodeGenerator::GenerateNode(int id, std::string& node_string) {
	std::stringstream lineStream_i(node_string);
	// x_1 y_1 z_1 zs_1 q_1 t_1
	double x, y, z, zs, q;
	int type;
	std::string ip;

	lineStream_i >> x;
	lineStream_i >> y;
	lineStream_i >> z;
	lineStream_i >> zs;
	lineStream_i >> q;
	lineStream_i >> type;
	lineStream_i >> ip;

	if(nodeParameters.count(type) > 0) {
		return new Node(id, x, y, z, zs, q, type, nodeParameters.at(type), ip);
	}
	else {
		// Bad type, just assume it is type 0
		return new Node(id, x, y, z, zs, q, type, nodeParameters.at(0), ip);
	}
}
