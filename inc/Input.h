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
#include <boost/numeric/conversion/cast.hpp>

#include "defines.h"
#include "FileReader.h"
#include "Node.h"
#include "NodeGenerator.h"
#include "Drone.h"
#include "DroneGenerator.h"


#define DEBUG_INPUT	DEBUG || 1

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
	Input(std::string scenario_input_path);
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
	// Get beta for drone l (planning energy budget, in Jule)
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
	void getTXParams_i(int i, double* a, double* b, double* mrate);
	// Get the "agnostic" max TX range for node i
	double getR_i(int i);
	// Get the x-coordinate of the base station
	double getX_b() { return mBaseStation.fX; }
	// Get the y-coordinate of the base station
	double getY_b() { return mBaseStation.fY; }
	// Get the z-coordinate of the base station
	double getZ_b() { return mBaseStation.fZ; }

protected:

private:
	// Path to input file
	std::string input_fileName;

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
};
