#include "TighteningHeuristic.h"


TighteningHeuristic::TighteningHeuristic() {}

// Attempts to improve hovering location positions using the tightening heuristic. Returns false if the solution would violate the drone's energy limit.
bool TighteningHeuristic::ImproveSubTour(int l, Input* input, std::vector<Point>* og_sub_tour, bool dummy_flag) {
	// Add the base station to the beginning and end of the tour
	std::vector<Point> sub_tour;
	{
		// Start at BS
		Point bs(-1, input->getX_b(), input->getY_b(), input->getZ_b());
		sub_tour.push_back(bs);
		// Add in the tour
		for(Point pt : *og_sub_tour) {
			sub_tour.push_back(pt);
		}
		// End at the base station
		sub_tour.push_back(bs);
	}

	int M_k = boost::numeric_cast<int>(sub_tour.size());

	if(DEBUG_HL_TH_LV1)
		printf("Running Tightening Heuristic\nSize of sub-tour: %d\n", M_k);

	/*
	 * Basic algorithm:
	 * 1. For each three consecutive stops; attempt to improve position of second stop
	 * 2. If we made a change is step 1, run repeat step 1
	 * 3. Verify drone will not run out of energy running final solution
	 *
	 * Given three stops, we have two to improve position of second stop:
	 * 1. Move second point onto line between points one and two
	 * 2. Move second point as close as possible to line between points one and two
	 */

	// Verify that we were given a vector with at least three points (start, single-stop, end)
	if(M_k < 3) {
		// Function not meant to run this way...
		fprintf(stderr,"[ERROR:TighteningHeuristic:ImproveSubTour] Given less than 3 points: %d\n", M_k);
		exit(1);
	}
	else if(M_k == 3) {
		if(DEBUG_HL_TH_LV1)
			printf("Only one stop\n");
		// Only one stop... need to move to point closest to the BS while at (or above) safety altitude
		double safety_altitude = input->getZ_i(sub_tour.at(1).node_id) + input->getZs_i(sub_tour.at(1).node_id);
		Point node2_point(sub_tour.at(1).node_id, input->getX_i(sub_tour.at(1).node_id), input->getY_i(sub_tour.at(1).node_id), input->getZ_i(sub_tour.at(1).node_id));
		// Check if we can stay at the BS
		Point zenith_at_bs(-1, sub_tour.at(0).x, sub_tour.at(0).y, std::max(safety_altitude, sub_tour.at(0).z));
		double dist_to_bs = node2_point.l2(zenith_at_bs);

		// Is this point close enough..?
		if(dist_to_bs <= input->getR_i(sub_tour.at(1).node_id)) {
			// Close enough, add it!
			sub_tour.at(1).x = zenith_at_bs.x;
			sub_tour.at(1).y = zenith_at_bs.y;
			sub_tour.at(1).z = zenith_at_bs.z;
			// Done!
			if(DEBUG_HL_TH_LV1)
				printf(" Point at BS zenith is close enough: ");
			zenith_at_bs.print();
		}
		else {
			// Set to point closest to zenith
			double R = input->getR_i(sub_tour.at(1).node_id);
			double Zs = input->getZs_i(sub_tour.at(1).node_id);
			double Rp = std::sqrt(R*R - Zs*Zs);
			Point zenith_zzero(-1, sub_tour.at(0).x, sub_tour.at(0).y, 0.0);
			Point node_zzero(sub_tour.at(1).node_id, input->getX_i(sub_tour.at(1).node_id), input->getY_i(sub_tour.at(1).node_id), 0);
			Point p_r = node_zzero.pointAtDistance(zenith_zzero, Rp);

			if(DEBUG_HL_TH_LV1)
				printf(" Using point on safety-plane(%f) ", Rp);
			p_r.print();

			sub_tour.at(1).x = p_r.x;
			sub_tour.at(1).y = p_r.y;
			sub_tour.at(1).z = safety_altitude;
		}
	}
	else {
		// More than one stop
		bool made_change = true;
		while(made_change) {
			made_change = false;
			if(DEBUG_HL_TH_LV1)
				printf("Start new iteration\n");
			/// For each three consecutive stops
			// For clarity, these points are hovering locations and not the actual nodes...
			std::vector<Point>::iterator p1 = sub_tour.begin();
			std::vector<Point>::iterator p2 = ++sub_tour.begin();
			std::vector<Point>::iterator p3 = ++++sub_tour.begin();
			while(p3 != sub_tour.end()) {
				if(DEBUG_HL_TH_LV2)
					printf(" Considering points: %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f)\n",
								p1->node_id, p1->x, p1->y, p1->z, p2->node_id, p2->x, p2->y, p2->z, p3->node_id, p3->x, p3->y, p3->z);

				// Measure the current distance...
				double current_dist = 0;
				Point prev = sub_tour.at(0);
				for(Point p : sub_tour) {
					current_dist += prev.l2(p);
					prev = p;
				}

				// Get information on the middle node
				int node2_id = p2->node_id;
				Node* node2 = input->getNode_i(node2_id);
				Point node2_point(node2_id, node2->getX(), node2->getY(), node2->getZ());

				/// Attempt to improve position of second stop
				// Find closest point to node2_point on line between points p1 and p3
				Point p_prime = closestPointBetweenLine(*p1, node2_point, *p3);
				// Set node id
				p_prime.node_id = node2_id;
				// Verify that this is at the minimum altitude
				p_prime.z = std::max((node2->getZ() + node2->getZs()), p_prime.z);

				if(DEBUG_HL_TH_LV2)
					printf("  Found point: ");
				p_prime.print();

				// Determine distance from node to this point
				double dist_to_node = p_prime.l2(node2_point);

				// Is this close enough to move the HL here?
				if(dist_to_node <= node2->getR()) {
					// Is this actually a difference..?
					if(floatEquality(p_prime.x, p2->x) && floatEquality(p_prime.y, p2->y) && floatEquality(p_prime.y, p2->y)) {
						// No
						if(DEBUG_HL_TH_LV2)
							printf("   New point sits at old point..\n");
					}
					else {
						// Measure this new distance
						double new_dist = 0;
						Point prev = sub_tour.at(0);
						// Cycle through all points
						for(Point p : sub_tour) {
							// Is this the middle point?
							if(p.node_id == p2->node_id) {
								// Yes, consider the new point p'
								new_dist += prev.l2(p_prime);
								prev = p_prime;
							}
							else {
								// No, just add in distance
								new_dist += prev.l2(p);
								prev = p;
							}
						}

						// Is this less than what we had before..?
						if(new_dist < current_dist) {
							// Yes, move the hovering location here!
							p2->x = p_prime.x;
							p2->y = p_prime.y;
							p2->z = p_prime.z;
							made_change = true;
							if(DEBUG_HL_TH_LV2)
								printf("   Move point! Now: %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f)\n",
											p1->node_id, p1->x, p1->y, p1->z, p2->node_id, p2->x, p2->y, p2->z, p3->node_id, p3->x, p3->y, p3->z);
						}
						else {
							if(DEBUG_HL_TH_LV2)
								printf("   This point does not reduce total distance\n");
						}
					}
				}
				else {
					if(DEBUG_HL_TH_LV2)
						printf("   Point is too far from node\n");

					// Move R meters down the vector from node2_point to p_prime_2
					Point p_r = node2_point.pointAtDistance(p_prime, input->getR_i(node2_id));

					// Is this point below the minimum allowed altitude?
					if(p_r.z < (node2->getZ() + node2->getZs())) {
						// This point is below the allowed height.. pick a new point on the plane of minimum altitude
						// Create new points all on the same plane
						Point p1_zzero(p1->node_id, p1->x, p1->y, 0);
						Point node2_p_zzero(node2_id, node2->getX(), node2->getY(), 0);
						Point p3_zzero(p3->node_id, p3->x, p3->y, 0);
						Point p_prime_zzero = closestPointBetweenLine(p1_zzero, node2_p_zzero, p3_zzero);
						// Determine new range on safe-altitude plane (R' = sqrt( R^2 - Zs^2 ) )
						double Rp = std::sqrt(node2->getR()*node2->getR() - node2->getZs()*node2->getZs());
						p_r = node2_p_zzero.pointAtDistance(p_prime_zzero, Rp);
						// Set the actual altitude
						p_r.z = node2->getZ() + node2->getZs();
					}

					// Set node id
					p_r.node_id = node2_id;

					if(DEBUG_HL_TH_LV2)
						printf("  Found point: ");
					p_r.print();


					// Is this actually a difference..?
					if(floatEquality(p_r.x, p2->x) && floatEquality(p_r.y, p2->y) && floatEquality(p_r.y, p2->y)) {
						// No
						if(DEBUG_HL_TH_LV2)
							printf("   Point p_r sits at old point..\n");
					}
					else {
						// Measure this new distance
						double new_dist = 0;
						Point prev = sub_tour.at(0);
						// Cycle through all points
						for(Point p : sub_tour) {
							// Is this the middle point?
							if(p.node_id == p2->node_id) {
								// Yes, consider the new point p_r
								new_dist += prev.l2(p_r);
								prev = p_r;
							}
							else {
								// No, just add in distance
								new_dist += prev.l2(p);
								prev = p;
							}
						}

						// Does this point reduce the total distance..?
						if(new_dist < current_dist) {
							// Yes, move the hovering location here!
							p2->x = p_r.x;
							p2->y = p_r.y;
							p2->z = p_r.z;
							made_change = true;
							if(DEBUG_HL_TH_LV1)
								printf("   Move to p_r! Now: %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f), %d(%.1f,%.1f,%.1f)\n",
									p1->node_id, p1->x, p1->y, p1->z, p2->node_id, p2->x, p2->y, p2->z, p3->node_id, p3->x, p3->y, p3->z);
						}
						else {
							if(DEBUG_HL_TH_LV2)
								printf("   p_r does not shorten total distance\n");
						}
					}
				}

				// Update points
				p1++;
				p2++;
				p3++;
			}
		}
	}

	if(DEBUG_HL_TH_LV1) {
		// Verify that we didn't mess something up...
		for(std::vector<Point>::iterator p = sub_tour.begin(); p != sub_tour.end(); p++) {
			// Make sure we have an actual node and not the bs...
			if(p->node_id >= 0) {
				// Create a point for this node
				Point node_point(p->node_id, input->getX_i(p->node_id), input->getY_i(p->node_id), input->getZ_i(p->node_id));
				// Get the distance from p to the node
				double dist = p->l2(node_point);
				if(dist > input->getR_i(p->node_id) + 0.0001) {
					fprintf(stderr, "[ERROR:TighteningHeuristic:ImproveSubTour] Distance from node too long (%f, should be <= %f)\n", dist, input->getR_i(p->node_id));
					exit(1);
				}
			}
		}
	}

	// Copy over the solution
	for(int og_sub_tour_i = 0, local_sub_tour_i = 1; og_sub_tour_i < boost::numeric_cast<int>(og_sub_tour->size()); og_sub_tour_i++, local_sub_tour_i++) {
		// Update this point's location
		if(og_sub_tour->at(og_sub_tour_i).node_id == sub_tour.at(local_sub_tour_i).node_id) {
			og_sub_tour->at(og_sub_tour_i).x = sub_tour.at(local_sub_tour_i).x;
			og_sub_tour->at(og_sub_tour_i).y = sub_tour.at(local_sub_tour_i).y;
			og_sub_tour->at(og_sub_tour_i).z = sub_tour.at(local_sub_tour_i).z;
		}
		else {
			fprintf(stderr, "[ERROR:TighteningHeuristic:ImproveSubTour] : local sub-tour order off from given sub-tour order (%d != %d)\n", og_sub_tour->at(og_sub_tour_i).node_id, sub_tour.at(local_sub_tour_i).node_id);
		}
	}


	// Calculate how much energy each action will take
	if(DEBUG_HL_TH_LV1)
		printf("Calculating energy used\n");

	double energy_used = 0.0;
	std::vector<Point>::iterator prev = sub_tour.begin();
	std::vector<Point>::iterator next = ++sub_tour.begin();
	// Loop through each top
	while(next != sub_tour.end()) {
		// Get the distance from point to point
		double dist_prv_nxt = prev->l2(*next);
		// Time to move point to point
		double time_prv_nxt = dist_prv_nxt/input->getV_l(l);
		// Add in energy to move point to point
		energy_used += input->getRho_m(l)*time_prv_nxt;

		if(DEBUG_HL_TH_LV2)
			printf(" Dist prv-nxt: %.2f, time: %.2f, energy: %.2f\n", dist_prv_nxt, time_prv_nxt, input->getRho_m(l)*time_prv_nxt);

		// Is the next point a node?
		if(next->node_id >= 0) {
			// Get the node that is being serviced
			Node* node_i = input->getNode_i(next->node_id);
			// Determine how long it takes to service this node
			double time_to_service = node_i->collectionTime(next->x, next->y, next->z);
			// Add in energy to service the node
			energy_used += input->getRho_h(l)*time_to_service;

			if(DEBUG_HL_TH_LV2)
				printf("  Service node %d, time: %.2f, energy: %.2f\n", next->node_id, time_to_service, input->getRho_h(l)*time_to_service);
		}

		// Update for loop
		prev = next;
		next++;
	}

	if(DEBUG_HL_TH_LV1)
		printf(" Energy used: %f, total budget: %f\n",energy_used, input->getB_l(l));

	return energy_used <= input->getB_l(l);
}


// Function to find the closest point p' on the line through Q and R to P bounded between Q and R
// Math found here: https://math.stackexchange.com/q/4657621
Point TighteningHeuristic::closestPointBetweenLine(const Point& Q, const Point& P, const Point& R) {
	// Direction vector from Q to R
	Point d = R - Q;

	// Vector from Q to P
	Point v = P - Q;

	// Compute t = (v . d) / (d . d)
	double t = v.dot(d) / d.dot(d);

	// Compute p' = Q + t * (R - Q)
	Point G = Q + d * t;

	// Direction of line QR
	Point D = R - Q;

	// How far G is from line QR (regardless of where it lies on the line)
	Point F = G - Q;

	// Hadamard division
	Point H = F / D;

	// Determine average of all non-zero elements
	double z = H.calc_z();

	if(z < 0) {
		// G was past Q
		return Q;
	}
	else if(z > 1) {
		// G was past R
		return R;
	}
	else {
		// G is in-fact between Q and R
		return G;
	}
}
