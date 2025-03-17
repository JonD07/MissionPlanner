/*
 * Solver_Standard.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 17, 2024
 *
 * Description: Standard online, local planner. This planner uses gurobi to optimize
 * hovering locations and expects a fixed ordering to the nodes to visit.
 */

#pragma once

#include <tuple>
#include <vector>
#include <string>

#include "Utilities.h"
#include "Solver.h"
#include "gurobi_c++.h"
#include "Online_Input.h"
#include "COptimizer.h"

#define DEBUG_SLVR_STD		DEBUG || 0

#define CONST_RELAXATION(X)		X,X+0.1


class Solver_Standard : public Solver, public COptimizer {
public:
	Solver_Standard(Constraint_tx_type constraint_type);

	void Solve(Input* input, Solution* I_final);

protected:
private:
};
