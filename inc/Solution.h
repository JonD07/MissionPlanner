/*
 * Solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 8, 2024
 *
 * Description:
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <complex>
#include <cstring>

#include "Input.h"

#define DEBUG_SOL	DEBUG || 0


struct HoveringLocation {
	double fX, fY, fZ;
	int nodeServiced;

	HoveringLocation(double x, double y, double z, int node) {
		fX = x;
		fY = y;
		fZ = z;
		nodeServiced = node;
	}
	HoveringLocation(const HoveringLocation& other) {
		fX = other.fX;
		fY = other.fY;
		fZ = other.fZ;
		nodeServiced = other.nodeServiced;
	}
	HoveringLocation& operator=(const HoveringLocation& other) {
		fX = other.fX;
		fY = other.fY;
		fZ = other.fZ;
		nodeServiced = other.nodeServiced;
		return *this;
	}
};


class Solution {
public:
	Solution(Input* input);
	virtual ~Solution();
	Solution(const Solution &other);
	Solution& operator=(const Solution &other);

	// Prints this solution
	void PrintSolution();
	// Prints plan file
	void PrintPlan();
	/*
	 * Determines the probability reward gained for the stored solution
	 */
	double Benchmark();
	// Determines if this is a valid assignment solution (doesn't break constraints)
	bool ValidSolution();
	// Place a hovering location into sub-tour k of drone l
	void AddHL(int l, int k, const HoveringLocation& hl);

private:
	Input* m_input;
	std::vector<std::vector<std::vector<HoveringLocation>>> tours_lkj;

	void setupEmptySolution();
};
