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
void Solution::PrintPlan(bool from_launch) {
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

				// Do we need to take off?
				if(from_launch) {
					// Take off
					fprintf(pOutputFile, "0 %f\n", START_AGL);
				}

				// For each hovering location..
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					Node* n = m_input->getNode_i(hl.nodeServiced);
					// Move to this location
					fprintf(pOutputFile, "1 %f %f %f 1.0\n", hl.fX, hl.fY, hl.fZ);
					// Service the node (cmd-5 node-x node-y node-z node-zs bytes-to-collect node-type node-ip)
					fprintf(pOutputFile, "5 %d %f %f %f %f %f %d %s\n", hl.nodeServiced, n->getX(), n->getY(), n->getZ(), n->getZs(), n->getQ()*1000000.0, n->getType(), n->getIP().c_str());
				}

				// Return home
				fprintf(pOutputFile, "2 %f\n", START_AGL);
				// Land
				fprintf(pOutputFile, "3\n");

				fclose(pOutputFile);
			}
		}
	}
}

/*
 *
 */
double Solution::Benchmark() {
	double total_time = 0;

	// Cycle through drones
	for(int l = 0; l < m_input->getM(); l++) {
		// Cycle through sub-tours for drone l
		for(int k = 0; k < m_input->getN(); k++) {
			// Does sub-tour k contain stops?
			if(tours_lkj.at(l).at(k).size() > 0) {
				// Is this a relaunch?
				if(k > 0) {
					// Yes, add in battery swap time
					total_time += m_input->getTb_l(l);
				}
				// Track the time it takes to run this tour
				double x_prev = m_input->getX_b(), y_prev = m_input->getY_b(), z_prev = m_input->getZ_b();

				// For each stop on this tour
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					// Determine the distance from point to point
					double dist_prv_nxt = distAtoB(x_prev, y_prev, z_prev, hl.fX, hl.fY, hl.fZ);
					// Time to travel this distance
					total_time += dist_prv_nxt/m_input->getV_l(l);

					// Determine node service time..
					Node* node_i = m_input->getNode_i(hl.nodeServiced);
					total_time += node_i->collectionTime(hl.fX, hl.fY, hl.fZ);
				}

				// Distance back to the base station
				double dist_prv_nxt = distAtoB(x_prev, y_prev, z_prev, m_input->getX_b(), m_input->getY_b(), m_input->getZ_b());
				// Time to travel back to bs
				total_time += dist_prv_nxt/m_input->getV_l(l);
			}
		}
	}

	return total_time;
}

/*
 * Fills sub_tours with the time to complete each sub-tour and gives a string for each sub-tour,
 * in the form l:k, where l is the drone and k is the sub-tour number for that drone.
 */
void Solution::GetSubTourTimes(std::vector<std::pair<std::string,double>>* sub_tours) {
	// Cycle through drones
	for(int l = 0; l < m_input->getM(); l++) {
		// Cycle through sub-tours for drone l
		for(int k = 0; k < m_input->getN(); k++) {
			// Does sub-tour k contain stops?
			if(tours_lkj.at(l).at(k).size() > 0) {
				double total_time = 0;
				// Is this a relaunch?
				if(k > 0) {
					// Yes, add in battery swap time
					total_time += m_input->getTb_l(l);
				}
				// Track the time it takes to run this tour
				double x_prev = m_input->getX_b(), y_prev = m_input->getY_b(), z_prev = m_input->getZ_b();

				// For each stop on this tour
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					// Determine the distance from point to point
					double dist_prv_nxt = distAtoB(x_prev, y_prev, z_prev, hl.fX, hl.fY, hl.fZ);
					// Time to travel this distance
					total_time += dist_prv_nxt/m_input->getV_l(l);

					// Determine node service time..
					Node* node_i = m_input->getNode_i(hl.nodeServiced);
					total_time += node_i->collectionTime(hl.fX, hl.fY, hl.fZ);
				}

				// Distance back to the base station
				double dist_prv_nxt = distAtoB(x_prev, y_prev, z_prev, m_input->getX_b(), m_input->getY_b(), m_input->getZ_b());
				// Time to travel back to bs
				total_time += dist_prv_nxt/m_input->getV_l(l);

				std::pair<std::string,double> tour(itos(l)+":"+itos(k), total_time);

				sub_tours->push_back(tour);
			}
		}
	}
}

/*
 * Determines if this is a valid assignment solution (doesn't break constraints).
 * We do this by checking to see if each node is visited and checking the total
 * energy used by each drone on each sub-tour.
 */
bool Solution::ValidSolution() {
	bool valid = true;

	// TODO: This!!

	return valid;
}

// Place a hovering location into sub-tour k of drone l
void Solution::AddHL(const HoveringLocation& hl, int l, int k) {
	if(l >= 0 && l < m_input->getM()) {
		if(k >= 0 && k < m_input->getN()) {
			tours_lkj.at(l).at(k).push_back(hl);
		}
	}
}

// Clears the current solution
void Solution::ClearSolution() {
	tours_lkj.clear();
	setupEmptySolution();
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

