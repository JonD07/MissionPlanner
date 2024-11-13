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
#include "COptimizer.h"
#include "TourImprover.h"


#define DEBUG_SLVR_VRP		DEBUG || 1


class Solver_VRP : public Solver {
public:
	Solver_VRP(TourImprover* improver);

	void Solve(Input* input, Solution* I_final);
	bool ImproveSubTour();

protected:
private:
	TourImprover* m_pTImprover;
};
