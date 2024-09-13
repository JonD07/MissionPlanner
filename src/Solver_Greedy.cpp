#include "Solver_Greedy.h"


Solver_Greedy::Solver_Greedy() {
	if(SANITY_PRINT)
		printf("Hello from Greedy Solver!\n");
	srand (time(NULL));
}


void Solver_Greedy::Solve(Input* input, Solution* I_crnt) {
	// Tracks what nodes are already visited
	std::vector<bool> nodes_visited;
	for(int i = 0; i < input->getN(); i++) {
		// Fill with false
		nodes_visited.push_back(false);
	}

	// Running solution
	std::vector<std::vector<std::vector<int>>> sub_tours_lk;
	// For each drone
	for(int l = 0; l < input->getM(); l++) {
		// Create drone's tour set
		std::vector<std::vector<int>> drone_tours_k;
		for(int k = 0; k < input->getN(); k++) {
			// Create a sub-tour
			std::vector<int> sub_tour;
			drone_tours_k.push_back(sub_tour);
		}
		sub_tours_lk.push_back(drone_tours_k);
	}

	bool increase_k = true;
	do {
		/// For each sub-tour index k
		for(int k = 0; k < input->getM(); k++) {
			/// For each drone
			for(int l = 0; l < input->getM(); l++) {
				// Set drone location
				double drone_x = input->getX_b(), drone_y = input->getY_b(), drone_z = input->getZ_b();
				/// While still adding nodes..
				while(true) {
					/// Find closest node
					double min_dist = std::numeric_limits<double>::max();
					int next_node = -1;
					for(int i = 0; i < input->getN(); i++) {
						// Is this node already covered?
						if(!nodes_visited.at(i)) {
							// Is this node closer than the current?
							double dist_to_node = distAtoB(drone_x, drone_y, drone_z, input->getX_i(i), input->getY_i(i), input->getZ_i(i));
							if(dist_to_node < min_dist) {
								// Found better candidate node
								next_node = i;
							}
						}
					}

					/// Verify energy or something here...
					if(next_node >= 0) {
						// Visit this node
						nodes_visited.at(next_node) = true;
						sub_tours_lk.at(l).at(k).push_back(next_node);
					}
					else {
						// Assumed to return to base here...
						break;
					}
				}
			}
		}

		// Determine if we visited all of the nodes
		bool all_visited = true;
		for(int i = 0; i < input->getN(); i++) {
			all_visited &= nodes_visited.at(i);
		}

		/// If all nodes were visited..
		if(all_visited) {
			// Found basic solution, stop increasing sub-tours
			increase_k = false;
		}
		else {
			// Not a valid solution, increase k
			increase_k = true;
			// Reset visited list
			for(int i = 0; i < input->getN(); i++) {
				nodes_visited.at(i) = false;
			}
			// Clear solution
			for(int l = 0; l < input->getM(); l++) {
				for(int k = 0; k < input->getN(); k++) {
					sub_tours_lk.at(l).at(k).clear();
				}
			}
		}
	} while(increase_k);

	// Print out what we found
	if(DEBUG_SLVR_GRDY) {
		printf("Found initial solution:\n");
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				int prev = -1;
				for(int i : sub_tours_lk.at(l).at(k)) {
					printf(" %d: %d -> %d on tour %d\n", l, prev, i, k);
					prev = i;
				}
			}
		}
	}

	/// Run hovering location optimizer
	HLOptimizer hlOptimizer;
	for(int l = 0; l < input->getM(); l++) {
		for(int k = 0; k < input->getN(); k++) {
			if(sub_tours_lk.at(l).at(k).size() > 1) {
				// Create a coordinates array
				std::vector<std::tuple<double,double,double>> coords;
				for(int i = 0; i < boost::numeric_cast<int>(sub_tours_lk.at(l).at(k).size()); i++) {
					std::tuple<double,double,double> coord;
					coords.push_back(coord);
				}
				// Run optimizer
				hlOptimizer.Optimize(l, input, &sub_tours_lk.at(l).at(k), &coords);
			}
		}
	}
}



