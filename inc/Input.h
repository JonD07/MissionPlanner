/*
 * Input.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 8, 2024
 *
 * Description: General Input class used to help process
 *  text file problem inputs.
 *
 *
 * Things that seem to be missing from the input..:
 *   1. Node data load (impacts time to collect data)
 *   2. Drone data..? Do we want heterogeneous drones? We at least need something for drone budgets
 *   3. Parameters to determine how long it will take to collect data from the node
 *
 */

#pragma once

#include <sstream>
#include <fstream>
#include <vector>

#include "defines.h"
#include "Node.h"

#define DEBUG_INPUT	DEBUG || 0

class Input {
public:
	Input(std::string input_path, int m);
	virtual ~Input();

	/// Getters
	// Number of nodes
	int getN() {return N;}
	// Number of drones
	int getM() {return M;}
	// Get the sensor range of node i
	double getR_i(int i);
	// Get the operational speed of drone l
	double getV_l(int l);
	// Get the time to swap batteries of drone l
	double getTb_l(int l);

protected:
	// Get next line from input an input file, ignores lines that start with '#'
	bool getNextLine(std::ifstream* file, std::string* line);


private:
	// Path to input file
	std::string input_fileName;

	// Number of nodes
	int N;
	// Number of drones
	int M;

	// Vector to hold all of the nodes in this graph
	std::vector<Node> vNodeLst;
	// Base station node
	Node mBaseStation;
};
