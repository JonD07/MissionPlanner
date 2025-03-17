#include "Solver_Greedy.h"


Solver_Greedy::Solver_Greedy(bool pwlApprx) {
	if(SANITY_PRINT)
		printf("Hello from Greedy Solver!\n");
	pwl_apprx = pwlApprx;
}


void Solver_Greedy::Solve(Input* input, Solution* I_crnt) {
	// Tracks what nodes are already visited
	std::vector<bool> nodes_visited;

	// Number of sub-tours per drone
	int k = 0;

	bool increase_k = true;
	while(increase_k) {
		// Increase k
		k++;
		increase_k = false;
		I_crnt->ClearSolution();
		nodes_visited.clear();
		for(int i = 0; i < input->getN(); i++) {
			// Fill with false
			nodes_visited.push_back(false);
		}

		// Number of drones (some are "fake" on future tours)
		int M_prime = input->getM()*k;

		// Running solution
		std::vector<std::vector<int>> sub_tours_l;

		// For each drone
		for(int l = 0; l < M_prime; l++) {
			// Create drone's tour set
			std::vector<int> drone_tours_k;
			sub_tours_l.push_back(drone_tours_k);
		}

		// Record where each drone is located
		std::vector<std::tuple<double,double,double>> drone_locals;
		std::vector<bool> drone_active;
		// Create l x k sub-tours
		for(int l = 0; l < M_prime; l++) {
			std::tuple<double,double,double> drone_local(input->getX_b(), input->getY_b(), input->getZ_b());
			drone_locals.push_back(drone_local);
			drone_active.push_back(true);
		}

		/// Fill drone task queues, in turn
		int l = 0;
		bool drones_active = true;
		do {
			if(drone_active.at(l)) {
				double drone_x = std::get<0>(drone_locals.at(l)), drone_y = std::get<1>(drone_locals.at(l)), drone_z = std::get<2>(drone_locals.at(l));
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
							min_dist = dist_to_node;
						}
					}
				}

				// Did we find a node to visit?
				if(next_node >= 0) {
					// Visit this node
					nodes_visited.at(next_node) = true;
					sub_tours_l.at(l).push_back(next_node);
					// Update drone position
					std::tuple<double,double,double> drone_local(input->getX_i(next_node), input->getY_i(next_node), input->getZ_i(next_node));
					drone_locals.at(l) = drone_local;
				}
				else {
					// Out of node, return to base...
					drone_active.at(l) = false;

					// Are there any other drones still active?
					drones_active = false;
					for(int l = 0; l < M_prime; l++) {
						drones_active |= drone_active.at(l);
					}
				}
			}

			l = (l+1)%(M_prime);
		} while(drones_active);

		/// Run hovering location optimizer
		HLOptimizer hlOptimizer;
		for(int l = 0; l < M_prime && !increase_k; l++) {
			if(sub_tours_l.at(l).size() > 0) {
				// Create a coordinates array
				std::vector<std::tuple<double,double,double>> coords;
				for(int i = 0; i < boost::numeric_cast<int>(sub_tours_l.at(l).size()); i++) {
					std::tuple<double,double,double> coord;
					coords.push_back(coord);
				}

				// Which drone is this actually?
				int l_actual = l % input->getM();

				// Run optimizer
				bool run_success = hlOptimizer.Optimize(l_actual, input, &sub_tours_l.at(l), &coords, pwl_apprx);

				// Did the optimizer succeed?
				if(run_success) {
					// Which sub-tour is this actually?
					int k_actual = int(l / input->getM());

					// Store the found solution
					for(int i = 0; i < boost::numeric_cast<int>(sub_tours_l.at(l).size()); i++) {
						HoveringLocation hl(std::get<0>(coords.at(i)), std::get<1>(coords.at(i)), std::get<2>(coords.at(i)), sub_tours_l.at(l).at(i));
						I_crnt->AddHL(hl, l_actual, k_actual);
					}
				}
				else {
					increase_k = true;
				}
			}
		}

		// Print out what we found
		if(DEBUG_SLVR_GRDY) {
			printf("Found initial solution:\n");
			for(int l = 0; l < input->getM(); l++) {
				for(int k = 0; k < input->getN(); k++) {
					int prev = -1;
					for(int i : sub_tours_l.at(l)) {
						printf(" %d: %d -> %d on tour %d\n", l, prev, i, k);
						prev = i;
					}
					if(prev >= 0) {
						printf(" %d: %d -> -1 on tour %d\n", l, prev, k);
					}
				}
			}
		}
	}



//	// Tracks what nodes are already visited
//	std::vector<bool> nodes_visited;
//	for(int i = 0; i < input->getN(); i++) {
//		// Fill with false
//		nodes_visited.push_back(false);
//	}
//
//	// Running solution
//	std::vector<std::vector<std::vector<int>>> sub_tours_lk;
//	// For each drone
//	for(int l = 0; l < input->getM(); l++) {
//		// Create drone's tour set
//		std::vector<std::vector<int>> drone_tours_k;
//		for(int k = 0; k < input->getN(); k++) {
//			// Create a sub-tour
//			std::vector<int> sub_tour;
//			drone_tours_k.push_back(sub_tour);
//		}
//		sub_tours_lk.push_back(drone_tours_k);
//	}
//
//	bool increase_k = true;
//	int k = 0;
//	do {
//		// Record where each drone is located
//		std::vector<std::tuple<double,double,double>> drone_locals;
//		std::vector<bool> drone_active;
//		for(int l = 0; l < input->getM(); l++) {
//			std::tuple<double,double,double> drone_local(input->getX_b(), input->getY_b(), input->getZ_b());
//			drone_locals.push_back(drone_local);
//			drone_active.push_back(true);
//		}
//
//		/// Fill drone task queues, in turn
//		int l = 0;
//		bool drones_active = true;
//		do {
//			if(drone_active.at(l)) {
//				double drone_x = std::get<0>(drone_locals.at(l)), drone_y = std::get<1>(drone_locals.at(l)), drone_z = std::get<2>(drone_locals.at(l));
//				/// Find closest node
//				double min_dist = std::numeric_limits<double>::max();
//				int next_node = -1;
//				for(int i = 0; i < input->getN(); i++) {
//					// Is this node already covered?
//					if(!nodes_visited.at(i)) {
//						// Is this node closer than the current?
//						double dist_to_node = distAtoB(drone_x, drone_y, drone_z, input->getX_i(i), input->getY_i(i), input->getZ_i(i));
//						if(dist_to_node < min_dist) {
//							// Found better candidate node
//							next_node = i;
//							min_dist = dist_to_node;
//						}
//					}
//				}
//
//				/// TODO: Verify energy or something here...
//				if(next_node >= 0) {
//					// Visit this node
//					nodes_visited.at(next_node) = true;
//					sub_tours_lk.at(l).at(k).push_back(next_node);
//					// Update drone position
//					std::tuple<double,double,double> drone_local(input->getX_i(next_node), input->getY_i(next_node), input->getZ_i(next_node));
//					drone_locals.at(l) = drone_local;
//				}
//				else {
//					// Assumed to return to base here...
//					drone_active.at(l) = false;
//
//					// Are there any other drones still active?
//					drones_active = false;
//					for(int l = 0; l < input->getM(); l++) {
//						drones_active |= drone_active.at(l);
//					}
//				}
//			}
//
//			l = (l+1)%input->getM();
//		} while(drones_active);
//
//		// Determine if we visited all of the nodes
//		bool all_visited = true;
//		for(int i = 0; i < input->getN(); i++) {
//			all_visited &= nodes_visited.at(i);
//		}
//
//		/// If there are still nodes that haven't been visited...
//		if(!all_visited) {
//			// increase k
//			increase_k = true;
//		}
//		else {
//			// Done!
//			increase_k = false;
//		}
//	} while(increase_k);
//
//	// Print out what we found
//	if(DEBUG_SLVR_GRDY) {
//		printf("Found initial solution:\n");
//		for(int l = 0; l < input->getM(); l++) {
//			for(int k = 0; k < input->getN(); k++) {
//				int prev = -1;
//				for(int i : sub_tours_lk.at(l).at(k)) {
//					printf(" %d: %d -> %d on tour %d\n", l, prev, i, k);
//					prev = i;
//				}
//				if(prev >= 0) {
//					printf(" %d: %d -> -1 on tour %d\n", l, prev, k);
//				}
//			}
//		}
//	}
//
//	/// Run hovering location optimizer
//	HLOptimizer hlOptimizer;
//	for(int l = 0; l < input->getM(); l++) {
//		for(int k = 0; k < input->getN(); k++) {
//			if(sub_tours_lk.at(l).at(k).size() > 1) {
//				// Create a coordinates array
//				std::vector<std::tuple<double,double,double>> coords;
//				for(int i = 0; i < boost::numeric_cast<int>(sub_tours_lk.at(l).at(k).size()); i++) {
//					std::tuple<double,double,double> coord;
//					coords.push_back(coord);
//				}
//
//				// Run optimizer
//				hlOptimizer.Optimize(l, input, &sub_tours_lk.at(l).at(k), &coords, pwl_apprx);
//
//				// Store the found solution
//				for(int i = 0; i < boost::numeric_cast<int>(sub_tours_lk.at(l).at(k).size()); i++) {
//					HoveringLocation hl(std::get<0>(coords.at(i)), std::get<1>(coords.at(i)), std::get<2>(coords.at(i)), sub_tours_lk.at(l).at(k).at(i));
//					I_crnt->AddHL(hl,l,k);
//				}
//			}
//		}
//	}
}



