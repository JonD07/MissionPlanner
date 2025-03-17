/*
 * Solver_Tabs.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 10, 2025
 *
 * Description: This planner uses Gurobi within the COptimizer with look-up tables
 * to optimize hovering locations and expects a fixed ordering to the nodes to visit.
 */

#pragma once

#include <tuple>
#include <vector>
#include <string>

#include "Utilities.h"
#include "Solver.h"
#include "gurobi_c++.h"
#include "Online_Input.h"

#define DEBUG_SLVR_STD		DEBUG || 0

#define CONST_RELAXATION(X)		X,X+0.1


class Solver_Tabs : public Solver {
public:
	Solver_Tabs();

	void Solve(Input* input, Solution* I_final);

protected:
private:
};
