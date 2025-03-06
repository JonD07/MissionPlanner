/*
 * COptimizer.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 8, 2024
 *
 * Description: Optimizer the hovering locations of a given sub-tour such
 * that the time to run the sub-tour is minimized.
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <list>
#include <time.h>
#include <utility>

#include "Utilities.h"
#include "Solver.h"
#include "TourImprover.h"
#include "gurobi_c++.h"




#define DEBUG_CV_OPTMZR		DEBUG || 1

#define CONST_RELAXATION(X)		X,X+0.1

// To easily adjust code for new constraint types
enum Constraint_tx_type {
	SINGLE_APPROXIMATION = 0,	//Single line approximation from lookup table
	PWL,						//Piecewise linear constraints
	TABULAR_CUT,				//Lazy constraint (TODO rename this to something more accurate.)
	INVERSE_SQUARE				//Uses the full inverse square law constraint
};


class COptimizer : public TourImprover {
public:
	COptimizer(Constraint_tx_type constraint_type);

	// Finds optimized hovering locations. Returns false if no solution found (hit drone energy limit)
	bool ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool aprx_tx_curve = false);

	//Generates Single Approximation constraint
	void GenerateSingleApproxConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	//Generates Lazy constraint
	void GenerateLazyConstraint(int l, GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	//Generates PWL (Piecewise linear) constraint
	void GeneratePWLConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	// // Helper function for directly solving the nonconvex problem
	void GenerateInverseSquareConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j, std::vector<GRBVar>* D2n_j);

	// Lookup distance
	double lookup_distance(int node_type, double q_size, double velocity);

	// Distance lookup table array for data_size and air speed to optimal distance approximation
	// distance_lookup[node_type][velocity][data_size]
	// node_type := pi3, pi4
	const static uint16_t NUM_NODE_TYPES = 2;
	// velocity := 2-20 [m/s] in increments of 1 m/s
	const static uint16_t NUM_VELOCITY_MEASUREMENTS = 19;
	// packet_size := .001 * eighth_root(2)^index [MB]
	const static uint16_t NUM_DATA_PACKAGE_SIZES = 160;
	// For example, the final entry for packet size is 0.001 * 2^(159/8) = 961.548431607 MB
	double pi_q_vs_distance_lookup[NUM_NODE_TYPES][NUM_VELOCITY_MEASUREMENTS][NUM_DATA_PACKAGE_SIZES];
protected:
private:
	Constraint_tx_type constraint_type;


};
