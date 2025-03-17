/*
 * Input.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 8, 2024
 *
 * Description: General Input class used to help process
 *  text file problem inputs.
 *
 */

#pragma once

#include <sstream>
#include <fstream>
#include <vector>
#include <boost/numeric/conversion/cast.hpp>

#include "defines.h"
#include "FileReader.h"
#include "Node.h"
#include "NodeGenerator.h"
#include "Drone.h"
#include "DroneGenerator.h"


#define DEBUG_INPUT	DEBUG || 0

struct BaseStation {
	double fX, fY, fZ;

	BaseStation(double x, double y, double z) {
		fX = x;
		fY = y;
		fZ = z;
	}
	BaseStation() {
		fX = 0.0;
		fY = 0.0;
		fZ = 0.0;
	}
};

class Input {
public:
	virtual ~Input();

	/// Getters
	// Number of nodes
	int getN() {return N;}
	// Number of drones
	int getM() {return M;}
	// Get the operational speed of drone l
	double getV_l(int l);
	// Get the time to swap batteries of drone l
	double getTb_l(int l);
	// Get rho for moving for drone l
	double getRho_m(int l);
	// Get rho for moving for drone l
	double getRho_h(int l);
	// Get beta for drone l (planning energy budget, in Jules)
	double getB_l(int l);
	// Get the x-coordinate of node i
	double getX_i(int i);
	// Get the y-coordinate of node i
	double getY_i(int i);
	// Get the z-coordinate of node i
	double getZ_i(int i);
	// Get node i's safe altitude
	double getZs_i(int i);
	// Get node i's data quantity to collect
	double getQ_i(int i);
	// Get i's node type
	int getNodeType_i(int i);
	// Get data TX parameters for node i
	void getTXParams_i(int i, double* a, double* b, double* max_rate, double* C, double* min_rate);
	// Get the "agnostic" max TX range for node i
	double getR_i(int i);
	// Lookup the optimal range for drone l to communicate with node i
	double getOptimalRange(int l, int i);
	// Get the x-coordinate of the base station
	double getX_b() { return mBaseStation.fX; }
	// Get the y-coordinate of the base station
	double getY_b() { return mBaseStation.fY; }
	// Get the z-coordinate of the base station (this is the starting altitude - start AGL + ground-z)
	double getZ_b() { return mBaseStation.fZ + START_AGL; }
	// Get the z-coordinate of the base station (this is the ground-z)
	double getZ_bg() { return mBaseStation.fZ; }
	// Get a pointer to node i
	Node* getNode_i(int i);

protected:
	// Protected contructor (we don't want people creating inputs, need to create a child class)
	Input(std::string scenario_input_path);

	// Read status
	bool read_success = true;

	// Path to input file
	std::string input_fileName;
	std::string node_line;
	std::string drone_line;
	std::string input_line;

	// The rest of the input
	std::vector<std::string> scenario_data;
	

	// Number of nodes
	int N;
	// Number of drones
	int M;

	// Vector to hold all of the nodes in this graph
	std::vector<Node*> vNodeLst;
	// Vector to hold all drones
	std::vector<Drone*> vDroneLst;
	// Base station node
	BaseStation mBaseStation;

private:
};
