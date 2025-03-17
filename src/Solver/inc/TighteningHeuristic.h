/*
 * TighteningHeuristic.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 28, 2024
 *
 * Description: Uses a variety of techniques to "tighten" hovering locations around the boundaries of sensor communication limits.
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


#define DEBUG_HL_TH_LV1		DEBUG || 0
#define DEBUG_HL_TH_LV2		DEBUG || 0


class TighteningHeuristic : public TourImprover {
public:
	TighteningHeuristic();

	// Attempts to improve hovering location positions using the tightening heuristic. Returns false if the solution would violate the drone's energy limit.
	bool ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool dummy_flag);

protected:
private:
	// Function to find the closest point p' on the line through Q and R to P bounded between Q and R
	// Math found here: https://math.stackexchange.com/q/4657621
	Point closestPointBetweenLine(const Point& Q, const Point& P, const Point& R);
};
