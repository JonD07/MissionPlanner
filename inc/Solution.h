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

#include "Input.h"

#define DEBUG_SOL	DEBUG || 0

using namespace std::complex_literals;

class Solution {
public:
	Solution(Input* input);
	virtual ~Solution();
	Solution(const Solution &other);
	Solution& operator=(const Solution &other);

	// Prints this solution
	void PrintSolution();
	// Assigns agent i to task j for task-slot k
	void Update(int i, int j, int k);
	/*
	 * Determines the probability reward gained for the stored solution
	 */
	double Benchmark();
	// Determines if this is a valid assignment solution (doesn't break constraints)
	bool ValidSolution();

	// Solution space - assigned tasks (NxM)
	bool*** X_ijk;

private:
	Input* m_input;
};
