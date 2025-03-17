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

struct max_float {
	double val;	// The value used for ordering in queue
	int ID;		// ID of this float

	max_float(double d, int id) : val(d), ID(id) {}
	max_float(const max_float& other) {
		val = other.val;
		ID = other.ID;
	}
	max_float& operator=(const max_float& other) {
		val = other.val;
		ID = other.ID;

	    return *this;
	}

	// Overload the operator < to ensure the priority queue orders by val
	bool operator<(const max_float& other) const {
		return val < other.val;
	}
};

class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(Input* input, Solution* I_final) = 0;
protected:
	bool wholeNumber(double f);
};
