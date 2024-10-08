#include "Solver_VRP.h"


Solver_VRP::Solver_VRP(bool pwlApprx) {
	if(SANITY_PRINT)
		printf("Hello from VRP Solver!\n");
	pwl_apprx = pwlApprx;
}


void Solver_VRP::Solve(Input* input, Solution* I_crnt) {
	/// Set k to zero
	int K = 0;
	bool increase_k = true;

	std::vector<std::vector<int>> ordered_subtours;
	std::vector<std::vector<int>> assignment_lk;
	std::vector<std::vector<kPoint>> cluster_k;

	// While we are still increasing k...
	while(increase_k) {
		// Reset solution
 		ordered_subtours.clear();
		assignment_lk.clear();
		for(int l = 0; l < input->getM(); l++){
			std::vector<int> clusters;
			assignment_lk.push_back(clusters);
		}
 		cluster_k.clear();

 		// Make a solution for this run
 		Solution currentSolution(input);

 		/// Increment K by the number of drones
 		K++;
 		increase_k = false;

 		// Verify we did not go over the limit...
		if(K > input->getN()) {
			fprintf(stderr, "[ERROR:Solver_VRP:Solve] More sub-tours (%d) than nodes (%d)\n", K, input->getN());
			exit(1);
		}

		/// Form k clusters
		// Put each node into a kPoint
		std::vector<kPoint> nodePoints;
		for(int i = 0; i < input->getN(); i++) {
			Node* n_i = input->getNode_i(i);
			kPoint pnt_i(i, 0, n_i->getX(), n_i->getY(), n_i->getZ());
			nodePoints.push_back(pnt_i);
		}

		if(DEBUG_SLVR_VRP) {
			printf("Running clustering algorithm\n");
		}

		// Run clustering algorithm
		ClusteringAlgorithm clusteringAlg;
		clusteringAlg.Solve(K, &nodePoints, &cluster_k);

		if(DEBUG_SLVR_VRP) {
			printf("Cluster:\n");
			for(std::vector<kPoint> cluster : cluster_k) {
				printf(" %d:", cluster.front().centroid_ID);
				for(kPoint p : cluster) {
					printf(" %d", p.point_ID);
				}
				printf("\n");
			}
		}

		/// Solve TSP on each cluster
		LKH_TSP_Solver tspSolver;
		for(int k = 0; k < K; k++) {
			// Create a vector with all of the stops (and the BS at the end)
			std::vector<Stop> vStops;
			// Fill vector with each point in the cluster
			for(kPoint point : cluster_k.at(k)) {
				Stop stp(point.point_ID, point.X, point.Y, point.Z);
				vStops.push_back(stp);
			}
			// Add base station as last stop
			{
				Stop stp(-1, input->getX_b(), input->getY_b(), input->getZ_b());
				vStops.push_back(stp);
			}

			// Create empty path vector (solver will fill this with the solution)
			std::vector<int> vPath;

			// Run TSP solver
			tspSolver.Solve_TSP(vStops, vPath);

			// Sanity print..
			if(DEBUG_SLVR_VRP) {
				printf("VRP solution:\n");
				for(int i : vPath) {
					printf(" %d: %d (%f, %f, %f)\n", i, vStops.at(i).ID, vStops.at(i).X, vStops.at(i).Y, vStops.at(i).Z);
				}
			}

			// Tour vector (these are the node's actual i values)
			std::vector<int> vTour_i;

			// We need to put node -1 first.. Find this node
			{
				int iteration = 0, nodes_found = 0;
				bool found_bs = false;
				while(true) {
					int i = iteration%boost::numeric_cast<int>(vStops.size());

					if(found_bs) {
						// We found the BS. Add this node to the tour
						vTour_i.push_back(vStops.at(vPath.at(i)).ID);
						nodes_found++;
					}
					else {
						// Still searching..
						if(vStops.at(vPath.at(i)).ID == -1) {
							// Found the base station!
							found_bs = true;
							nodes_found++;
						}
					}

					// Have we found all of the stops?
					if(nodes_found >= boost::numeric_cast<int>(vPath.size())) {
						break;
					}

					iteration++;
				}
			}

			// Sanity print..
			if(DEBUG_SLVR_VRP) {
				printf("Fixed solution:\n");
				for(int i : vTour_i) {
					printf(" %d (%f, %f, %f)\n", i, input->getX_i(i), input->getY_i(i), input->getZ_i(i));
				}
			}

			// Store the final solution
			ordered_subtours.push_back(vTour_i);
		}

		if(DEBUG_SLVR_VRP) {
			printf("Assigning drones to sub-tours\n");
		}

		/// Form drone-to-sub-tour assignments
		// Not so nice solution...
		for(int k = 0; k <= K/input->getM(); k++) {
			// Used to rank sub-tour distances
			std::priority_queue<max_float> tour_distance_queue;
			// Used to rank drone max distances
			std::priority_queue<max_float> drone_budget_queue;
			int base_cluster_index = k*input->getM();
			int sub_tours_remaining = ordered_subtours.size() - base_cluster_index;

			if(DEBUG_SLVR_VRP) {
				printf(" sub-tour set %d\n", k);
			}

			// Rank tour distances
			for(int k_prime = 0; k_prime < input->getM() && k_prime < sub_tours_remaining; k_prime++) {
				double tour_dist = 0;
				double last_x = input->getX_b(), last_y = input->getY_b(), last_z = input->getZ_b();
				// For each node in this tour...
				for(int i : ordered_subtours.at(base_cluster_index+k_prime)) {
					// Add up the distance from node to node
					tour_dist += distAtoB(last_x, last_y, last_z, input->getX_i(i), input->getY_i(i), input->getZ_i(i));
					last_x = input->getX_i(i);
					last_y = input->getY_i(i);
					last_z = input->getZ_i(i);
				}
				// Add in the distance back to the base
				tour_dist += distAtoB(last_x, last_y, last_z, input->getX_b(), input->getY_b(), input->getZ_b());

				// Create sub-tour distance struct
				max_float st_dist(tour_dist, base_cluster_index+k_prime);

				// Store distance in priority queue
				tour_distance_queue.push(st_dist);
			}

			// Rank drone max distances
			for(int l = 0; l < input->getM(); l++) {
				max_float budget(input->getB_l(l), l);
				drone_budget_queue.push(budget);
			}

			// For each sub-tour
			for(int k_prime = 0; k_prime < input->getM() && k_prime < sub_tours_remaining; k_prime++) {
				// Pop off the top sub-tour and drone
				max_float max_dist_tour = tour_distance_queue.top();
				tour_distance_queue.pop();
				max_float strongest_drone = drone_budget_queue.top();
				drone_budget_queue.pop();

				if(DEBUG_SLVR_VRP) {
					printf("  drone %d (%f) goes on tour %d (%f)\n", strongest_drone.ID, strongest_drone.val, max_dist_tour.ID, max_dist_tour.val);
				}

				// Record that drone l will do cluster k
				assignment_lk.at(strongest_drone.ID).push_back(max_dist_tour.ID);
			}
		}

		/// Run child class sub-tour improvement algorithm
		bool valid_sub_tour = true;
		// For each drone
		for(int l = 0; l < input->getM() && valid_sub_tour; l++) {
			int drones_k = 0;
			// For each sub-tour that this drone does
			for(int k : assignment_lk.at(l)) {
				std::vector<int> sub_tour;
				std::vector<std::tuple<double,double,double>> coords;
				// For each node in this tour
				for(int i : ordered_subtours.at(k)) {
					// Create a coordinate to hold final solution
					std::tuple<double,double,double> coord;
					coords.push_back(coord);
					sub_tour.push_back(i);
				}

				// Run optimizer
				HLOptimizer hlOptimizer;
				valid_sub_tour &= hlOptimizer.Optimize(l, input, &sub_tour, &coords, false);

				if(valid_sub_tour) {
					if(DEBUG_SLVR_VRP) {
						printf("Good sub-tour %d:%d\n Adding hoving points:\n", l, drones_k);
					}

					// Store the found solution
					for(int i = 0; i < boost::numeric_cast<int>(ordered_subtours.at(k).size()); i++) {
						HoveringLocation hl(std::get<0>(coords.at(i)), std::get<1>(coords.at(i)), std::get<2>(coords.at(i)), ordered_subtours.at(k).at(i));
						currentSolution.AddHL(hl,l,drones_k);

						if(DEBUG_SLVR_VRP) {
							printf("  (%.3f, %.3f, %.3f) for node %d (%.3f, %.3f, %.3f)\n", hl.fX, hl.fY, hl.fZ, hl.nodeServiced, input->getX_i(hl.nodeServiced), input->getY_i(hl.nodeServiced), input->getZ_i(hl.nodeServiced));
						}
					}
				}
				else {
					if(DEBUG_SLVR_VRP) {
						printf("Bad sub-tour %d:%d!\n", l, drones_k);
					}
				}
				drones_k++;
			}
		}

		if(!valid_sub_tour || !currentSolution.ValidSolution()) {
			///   increase k...
			increase_k = true;
			if(DEBUG_SLVR_VRP) {
				printf("Increase k\n");
			}
		}
		else {
			// Benchmark current solution
			double currentObjective = currentSolution.Benchmark();

			// Is this better than the current solution?
			if(currentObjective < INF && currentObjective < I_crnt->Benchmark()) {
				if(DEBUG_SLVR_VRP) {
					printf("K = %d, found solution (%.3f) better than previous solution (%.3f)\n", K, currentObjective, I_crnt->Benchmark());
				}

				// Update the solution!
				I_crnt->UpdateSolution(&currentSolution);
				// Keep going, increasing k may help improve the solution
				increase_k = true;
			}
			else if(currentObjective < INF) {
				// No longer improving solution..
				increase_k = false;
				if(DEBUG_SLVR_VRP) {
					printf("No longer improving solution.. Found solution (%.f) > incumbent (%.3f)\n** Algorithm Ended **\n", currentObjective, I_crnt->Benchmark());
				}
			}
		}
	}
}



