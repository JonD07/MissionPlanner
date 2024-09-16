#include "Solution.h"

Solution::Solution(Input* input) {
	// Initialize the size information
	m_input = input;
	setupEmptySolution();

	if(DEBUG_SOL)
		printf("New Solution from Input: N = %d, M = %d\n", m_input->getN(), m_input->getM());
}

Solution::Solution(const Solution &other) {
	// Initialize the size information
	m_input = other.m_input;
	setupEmptySolution();
}

Solution& Solution::operator=(const Solution &other) {
	// Initialize the size information
	m_input = other.m_input;
	setupEmptySolution();

	return *this;
}

Solution::~Solution() {}


// Prints this solution
void Solution::PrintSolution() {
	for(int l = 0; l < m_input->getM(); l++) {
		for(int k = 0; k < m_input->getN(); k++) {
			int prev = -1;
			for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
				printf(" %d:%d %d->%d (%f, %f, %f)\n", l, k, prev, hl.nodeServiced, hl.fX, hl.fY, hl.fZ);
				prev = hl.nodeServiced;
			}
			if(prev >= 0) {
				printf(" %d:%d %d->BS\n", l, k, prev);
			}
		}
	}
}

/*
 * TODO: Determines the probability reward gained for the stored solution
 */
double Solution::Benchmark() {
	return 100.0;
}

// TODO: Determines if this is a valid assignment solution (doesn't break constraints)
bool Solution::ValidSolution() {
	/// Check each set of constraints
	bool valid = true;

	return valid;
}

// Place a hovering location into sub-tour k of drone l
void Solution::AddHL(int l, int k, const HoveringLocation& hl) {
	if(l >= 0 && l < m_input->getM()) {
		if(k >= 0 && k < m_input->getN()) {
			tours_lkj.at(l).at(k).push_back(hl);
		}
	}
}

void Solution::setupEmptySolution() {
	for(int l = 0; l < m_input->getM(); l++) {
		std::vector<std::vector<HoveringLocation>> drone_l;
		for(int k = 0; k < m_input->getN(); k++) {
			std::vector<HoveringLocation> sub_tour_k;
			drone_l.push_back(sub_tour_k);
		}
		tours_lkj.push_back(drone_l);
	}
}

