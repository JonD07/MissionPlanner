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

// Prints plan file
void Solution::PrintPlan() {
	// Cycle through drones
	for(int l = 0; l < m_input->getM(); l++) {
		// Cycle through sub-tours for drone l
		for(int k = 0; k < m_input->getN(); k++) {
			// Does sub-tour k contain stops?
			if(tours_lkj.at(l).at(k).size() > 0) {
				/// Create a plan file for this stop
				// Open a new plan file
				FILE * pOutputFile;
				char buff[100];
				std::string outputPath = "plan/";
				sprintf(buff, "%s", outputPath.c_str());
				sprintf(buff + strlen(buff), "plan_%d_%d.pln", l, k);
				if(SANITY_PRINT)
					printf(" Printing plan-%d-%d to: %s\n", l, k, buff);
				pOutputFile = fopen(buff, "w");

				// Take off
				fprintf(pOutputFile, "0 10\n");

				// For each hovering location..
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					// Move to this location
					fprintf(pOutputFile, "1 %f %f %f 1.0\n", hl.fX, hl.fY, hl.fZ);
					// Service the node
					fprintf(pOutputFile, "5 %d %f %f %f %f %f %d\n", hl.nodeServiced, hl.fX, hl.fY, hl.fZ, m_input->getZs_i(hl.nodeServiced), m_input->getQ_i(hl.nodeServiced), m_input->getNodeType_i(hl.nodeServiced));
				}

				// Return home
				fprintf(pOutputFile, "2 10\n");
				// Land
				fprintf(pOutputFile, "3\n");


				fclose(pOutputFile);

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

