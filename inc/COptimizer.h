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


class COptimizer : public TourImprover {
public:
	COptimizer(uint16_t constraint_type);

	// Finds optimized hovering locations. Returns false if no solution found (hit drone energy limit)
	bool ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool aprx_tx_curve = false);

	//TODO Description once you know what it does
	// callback_class callback_object;

protected:
private:
	bool pwa_tx_curve;
};
