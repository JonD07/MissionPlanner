/*
 * Solver_Baseline.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 28, 2024
 *
 * Description: Clustering followed by TSP. This uses an TX-agnostic approach to improve performance (the tightening heuristic).
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
#include "TighteningHeuristic.h"
#include "ClusteringAlgorithm.h"
#include "LKH_TSP_Solver.h"


#define DEBUG_SLVR_BL		DEBUG || 0


class Solver_Baseline : public Solver {
public:
	Solver_Baseline(bool heuristic = true);

	void Solve(Input* input, Solution* I_final);

protected:
private:
	bool use_heuristic;
};
