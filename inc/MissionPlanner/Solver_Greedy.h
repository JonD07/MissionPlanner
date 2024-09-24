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
	Solver_Greedy(bool pwlApprx = false);

	void Solve(Input* input, Solution* I_final);

protected:
private:
	// Used to for a peice-wise linear approximation of the TX curve (not always fast...)
	bool pwl_apprx;
};
