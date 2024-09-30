/*
 * Solver_VRP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 24, 2024
 *
 * Description: Clustering followed by TSP.
 */

#pragma once

#include <tuple>
#include <vector>
#include <string>
#include <list>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "HLOptimizer.h"
#include "ClusteringAlgorithm.h"
#include "LKH_TSP_Solver.h"


#define DEBUG_SLVR_VRP		DEBUG || 0


class Solver_VRP : public Solver {
public:
	Solver_VRP(bool pwlApprx = false);

	void Solve(Input* input, Solution* I_final);

protected:
private:
	// Used to for a peice-wise linear approximation of the TX curve (not always fast...)
	bool pwl_apprx;
};
