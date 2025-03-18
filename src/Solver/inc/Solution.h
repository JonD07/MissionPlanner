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
#include <utility>
#include <limits>

#include "Input.h"
#include "Utilities.h"

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
	void PrintPlan(bool add_launch = true, bool add_land = true, std::string outputPath = "plan/");
	/*
	 * Calculate the objective function value for this solution
	 */
	double Benchmark();
	/*
	 * Calculate the max latency
	 */
	double MaxLatency();
	/*
	 * Fills sub_tours with the time to complete each sub-tour and gives a string for each sub-tour,
	 * in the form l:k, where l is the drone and k is the sub-tour number for that drone.
	 */
	void GetSubTourTimes(std::vector<std::pair<std::string,double>>* sub_tours);
	/*
	 * Fills the sub_tours array with strings for each sub-tour. The string will be the following format:
	 * l:k:i-i-i...-i, where l is the drone's id, k is the drones sub-tour number, and i- ... -i are the
	 * stops on that tour.
	 */
	void GetSubTours(std::vector<std::string>* sub_tours);
	/*
	 * Determines if this is a valid assignment solution (doesn't break constraints).
	 * We do this by checking to see if each node is visited and checking the total
	 * energy used by each drone on each sub-tour.
	 */
	bool ValidSolution(bool print_errors = false);
	/*
	 * Determines how much energy drone l will use if it completes the given tour
	 */
	double CalculateEnergy(int l, const std::vector<HoveringLocation>& tour);
	// Place a hovering location into sub-tour k of drone l
	void AddHL(const HoveringLocation& hl, int l = 0, int k = 0);
	// Clears out any save solution in this class and imports the solution stored in other
	void UpdateSolution(const Solution* other);
	// Clears the current solution
	void ClearSolution();

private:
	Input* m_input;
	std::vector<std::vector<std::vector<HoveringLocation>>> tours_lkj;

	void setupEmptySolution();
};
