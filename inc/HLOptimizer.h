/*
 * HLOptimizer.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 12, 2024
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
#include "gurobi_c++.h"

#define DEBUG_HL_OPTMZR		DEBUG || 1

#define CONST_RELAXATION(X)		X,X+0.1


class HLOptimizer {
public:
	HLOptimizer();

	void Optimize(int l, Input* input, const std::vector<int>* sub_tour, std::vector<std::tuple<double,double,double>>* cords);

protected:
private:
};
