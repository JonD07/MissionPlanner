/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 8, 2024
 *
 * Description: Parent class for all solvers
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <complex>

#include "Input.h"
#include "Utilities.h"
#include "Solution.h"

#define DEBUG_SOLVER	0 || DEBUG

class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(Input* input, Solution* I_final) = 0;
protected:
	bool wholeNumber(double f);
};
