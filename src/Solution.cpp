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
 * Calculates the objective function values for the stored solution. Will return INF if this is not a valid solution.
 */
double Solution::Benchmark() {
	if(ValidSolution()) {
		double total_time = 0;
		std::vector<int> drones_used;
		for(int l = 0; l < m_input->getM(); l++) {
			drones_used.push_back(0);
		}

		// Cycle through drones
		for(int l = 0; l < m_input->getM(); l++) {
			// Cycle through sub-tours for drone l
			for(int k = 0; k < m_input->getN(); k++) {
				// Does sub-tour k contain stops?
				if(tours_lkj.at(l).at(k).size() > 0) {
					// Mark that this drone runs a tour
					drones_used.at(l) = 1;

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

						// Update position
						x_prev = hl.fX;
						y_prev = hl.fY;
						z_prev = hl.fZ;
					}

					// Distance back to the base station
					double dist_prv_nxt = distAtoB(x_prev, y_prev, z_prev, m_input->getX_b(), m_input->getY_b(), m_input->getZ_b());
					// Time to travel back to bs
					total_time += dist_prv_nxt/m_input->getV_l(l);
				}
			}
		}

		// How many drones were actually deployed?
		int drones_deployed = 0;
		for(int l = 0; l < m_input->getM(); l++) {
			drones_deployed += drones_used.at(l);
		}

		return total_time/drones_deployed;
	}
	else {
		return std::numeric_limits<double>::max();
	}
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
 * Fills the sub_tours array with strings for each sub-tour. The string will be the following format:
 * l:k:i-i-i...-i, where l is the drone's id, k is the drones sub-tour number, and i- ... -i are the
 * stops on that tour.
 */
void Solution::GetSubTours(std::vector<std::string>* sub_tours) {
	// Cycle through drones
	for(int l = 0; l < m_input->getM(); l++) {
		// Cycle through sub-tours for drone l
		for(int k = 0; k < m_input->getN(); k++) {
			// Does sub-tour k contain stops?
			if(tours_lkj.at(l).at(k).size() > 0) {
				// Create a string for this sub-tour
				std::string tour_string = itos(l)+":"+itos(k)+":";

				// For each stop on this tour
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					tour_string += itos(hl.nodeServiced)+"-";
				}

				// Add string to array
				sub_tours->push_back(tour_string);
			}
		}
	}
}

/*
 * Determines if this is a valid assignment solution (doesn't break constraints).
 * We do this by checking to see if each node is visited and checking the total
 * energy used by each drone on each sub-tour.
 */
bool Solution::ValidSolution(bool print_errors) {
	bool valid_solution = true;

	// Verify that each sub-tour is visited at least once
	std::vector<bool> visited;
	for(int i = 0; i < m_input->getN(); i++) {
		visited.push_back(false);
	}

	// Cycle through all tours (start with drones)
	for(int l = 0; l < m_input->getM(); l++) {
		// Cycle through sub-tours for drone l
		for(int k = 0; k < m_input->getN(); k++) {
			// Does sub-tour k contain stops?
			if(tours_lkj.at(l).at(k).size() > 0) {
				// Check that we don't go over energy here
				double tour_energy = CalculateEnergy(l, tours_lkj.at(l).at(k));
				if(tour_energy > m_input->getB_l(l)) {
					// We went over...
					valid_solution = false;
					if(print_errors) {
						fprintf(stderr,"[ERROR:ValidSolution] Sub-tour %d:%d is over energy budget! %.3f > %.3f\n", l, k, tour_energy, m_input->getB_l(l));
					}
				}

				// Check each stop on this sub-tour
				for(HoveringLocation hl : tours_lkj.at(l).at(k)) {
					if(hl.nodeServiced >= 0 && hl.nodeServiced < m_input->getN()) {
						visited.at(hl.nodeServiced) = true;
					}
				}
			}
		}
	}

	// Did we hit every node?
	for(int i = 0; i < m_input->getN(); i++) {
		if(!visited.at(i)) {
			valid_solution = false;
			if(print_errors) {
				fprintf(stderr,"[ERROR:ValidSolution] Missed node %d\n", i);
			}
		}
	}

	return valid_solution;
}

/*
 * Determines how much energy drone l will use if it completes the given tour
 */
double Solution::CalculateEnergy(int l, const std::vector<HoveringLocation>& tour) {
	double total_energy = 0;
	const double drone_speed = m_input->getV_l(l);

	// Set current position (assume that we start at the base station)
	double x = m_input->getX_b(), y = m_input->getY_b(), z = m_input->getZ_b();
	// Cycle through stops in tour
	for(HoveringLocation hl : tour) {
		// Get distance from our current position to the next hl
		double dist_a_b = distAtoB(x, y, z, hl.fX, hl.fY, hl.fZ);
		// Calculate time
		double time_a_b = dist_a_b/drone_speed;
		// Add in energy
		total_energy += m_input->getRho_m(l)*time_a_b;

		// Update position
		x = hl.fX;
		y = hl.fY;
		z = hl.fZ;

		// Determine time spent hovering
		double service_time = m_input->getNode_i(hl.nodeServiced)->collectionTime(x,y,z);
		// Add in energy
		total_energy += m_input->getRho_h(l)*service_time;
	}
	// Add in energy to get back to the base station
	{
		double dist_i_base = distAtoB(x, y, z, m_input->getX_b(), m_input->getY_b(), m_input->getZ_b());
		// Calculate time
		double time_i_base = dist_i_base/drone_speed;
		// Add in energy
		total_energy += m_input->getRho_m(l)*time_i_base;
	}

	return total_energy;
}

// Place a hovering location into sub-tour k of drone l
void Solution::AddHL(const HoveringLocation& hl, int l, int k) {
	if(l >= 0 && l < m_input->getM()) {
		if(k >= 0 && k < m_input->getN()) {
			tours_lkj.at(l).at(k).push_back(hl);
		}
	}
}

// Clears out any save solution in this class and imports the solution stored in other
void Solution::UpdateSolution(const Solution* other) {
	// Clear any current solution
	ClearSolution();

	/// Add in the other solution
	// For-each drone
	for(int l = 0; l < m_input->getM(); l++) {
		// For-each sub-tour
		for(int k = 0; k < m_input->getN(); k++) {
			// Move hovering locations for l:k over to this solution
			for(HoveringLocation hl : other->tours_lkj.at(l).at(k)) {
				AddHL(hl,l,k);
			}
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

