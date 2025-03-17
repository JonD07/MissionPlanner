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
#include "q_vs_distance.h"




#define DEBUG_CV_OPTMZR		DEBUG || 0

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
protected:

	//Generates Single Approximation constraint
	void GenerateSingleApproxConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	//Generates Lazy constraint
	void GenerateLazyConstraint(int l, GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	//Generates PWL (Piecewise linear) constraint
	void GeneratePWLConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j);

	// // Helper function for directly solving the nonconvex problem
	void GenerateInverseSquareConstraint(GRBModel &model, std::vector<Point>* sub_tour, Input* input, std::vector<GRBVar>* R_j, std::vector<GRBVar>* Dn_j, std::vector<GRBVar>* D2n_j);

	Constraint_tx_type constraint_type;
private:
};
