/*
 * Solver_Greedy.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 12, 2024
 *
 * Description: Basic greedy solver. This just assigns drones to nodes somewhat randomly
 * then runs the hovering location optimizer. This was made to test the hovering location
 * optimizer rather than being a serious solution.
 */

#pragma once

#include <tuple>
#include <vector>
//#include <iostream>
#include <string>
//#include <limits>
//#include <list>
//#include <time.h>

#include "Utilities.h"
#include "Solver.h"
#include "HLOptimizer.h"

#define DEBUG_SLVR_GRDY		DEBUG || 1


class Solver_Greedy : public Solver {
public:
	Solver_Greedy();

	void Solve(Input* input, Solution* I_final);

protected:
private:
};
