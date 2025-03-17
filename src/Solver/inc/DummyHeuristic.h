/*
 * DummyHeuristic.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 8, 2024
 *
 * Description: This is a dummy tour improver heuristic. It does nothing to improve the original
 * solution, it is just used to show how the other heuristics impact performance.
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
#include "TourImprover.h"


#define DEBUG_DH		DEBUG || 0


class DummyHeuristic : public TourImprover {
public:
	DummyHeuristic();

	// Does nothing to improve sub-tour, just returns true.
	bool ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool dummy_flag);

protected:
private:
};
