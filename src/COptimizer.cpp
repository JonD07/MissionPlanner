#include "COptimizer.h"
#include "COptimizer_callback.h"


/*
 * Convex optimization approach to improving sub-tours. The pwa_flag tells the solver to use
 * a piece-wise approximation of the TX curve (this is NOT convex).
 */
COptimizer::COptimizer(bool pwa_flag) : pwa_tx_curve(pwa_flag) {}


// Finds optimized hovering locations. Returns false if no solution found (hit drone energy limit)
bool COptimizer::ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool aprx_tx_curve) {
	aprx_tx_curve |= pwa_tx_curve;

	int M_k = boost::numeric_cast<int>(sub_tour->size());
	try {
		//
		/// Create an environment
		//
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", "mip1.log");
		env.start();
		GRBModel model = GRBModel(env);
		if(aprx_tx_curve) {
			model.set(GRB_IntParam_NonConvex, 2);
			model.set(GRB_DoubleParam_TimeLimit, 500.0);
		} else{
			model.set(GRB_IntParam_LazyConstraints, 1);
		}

		if(DEBUG_CV_OPTMZR) {
			printf("Starting up Gurobi\n");
		}
		else {
			env.set(GRB_INT_PAR_OUTPUTFLAG, "0");
		}

		//
		/// Create variables
		//

		// Create position variables for each waypoint
		std::vector<GRBVar> X_j;
		std::vector<GRBVar> Y_j;
		std::vector<GRBVar> Z_j;

		for(int j = 0; j < M_k; j++) {
			int i = sub_tour->at(j).node_id;
			GRBVar x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + itos(j));
			GRBVar y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + itos(j));
			GRBVar z = model.addVar(input->getZ_i(i) + input->getZs_i(i), GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_" + itos(j));
			x.set(GRB_DoubleAttr_Start, input->getX_i(i));
			y.set(GRB_DoubleAttr_Start, input->getY_i(i));
			z.set(GRB_DoubleAttr_Start, input->getZ_i(i) + input->getZs_i(i));

			X_j.push_back(x);
			Y_j.push_back(y);
			Z_j.push_back(z);
		}

		// Create position variables for each node (relaxed on node position)
		std::vector<GRBVar> Xn_j;
		std::vector<GRBVar> Yn_j;
		std::vector<GRBVar> Zn_j;
		for(int j = 0; j < M_k; j++) {
			int i = sub_tour->at(j).node_id;
			GRBVar x = model.addVar(CONST_RELAXATION(input->getX_i(i)), 0.0, GRB_CONTINUOUS, "xn_" + itos(j));
			GRBVar y = model.addVar(CONST_RELAXATION(input->getY_i(i)), 0.0, GRB_CONTINUOUS, "yn_" + itos(j));
			GRBVar z = model.addVar(CONST_RELAXATION(input->getZ_i(i)), 0.0, GRB_CONTINUOUS, "zn_" + itos(j));

			Xn_j.push_back(x);
			Yn_j.push_back(y);
			Zn_j.push_back(z);
		}

		// Create position variables for base station
		GRBVar X_b = model.addVar(CONST_RELAXATION(input->getX_b()), 0.0, GRB_CONTINUOUS, "x_b");
		GRBVar Y_b = model.addVar(CONST_RELAXATION(input->getY_b()), 0.0, GRB_CONTINUOUS, "y_b");
		GRBVar Z_b = model.addVar(CONST_RELAXATION(input->getZ_b()), 0.0, GRB_CONTINUOUS, "z_b");

		// Create distance i -> j variables
		std::vector<GRBVar> Di_j;
		for(int j = 0; j < M_k; j++) {
			// Distance from waypoint i to j
			GRBVar d = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "di_" + itos(j));
			Di_j.push_back(d);
		}

		// Create distance j -> bs variable
		GRBVar Dnb = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "Dnb");

		// Create distance node_i -> wp_i variables
		std::vector<GRBVar> Dn_j;
		for(int j = 0; j < M_k; j++) {
			// Distance from node i to waypoint i
			GRBVar d = model.addVar(0.1, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "dn_" + itos(j));
			Dn_j.push_back(d);
		}

		// Create service time variable
		std::vector<GRBVar> Ts_j;
		for(int j = 0; j < M_k; j++) {
			// Sequence number for waypoint i
			GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "ts_" + itos(j));
			Ts_j.push_back(t);
		}

		// Create TX rate variable
		std::vector<GRBVar> R_j;
		for(int j = 0; j < M_k; j++) {
			int i = sub_tour->at(j).node_id;
			// Get battery details
			double a, b, mrate, c;
			input->getTXParams_i(i, &a, &b, &mrate, &c);
			// Sequence number for waypoint i
			GRBVar r = model.addVar(0.0, mrate, 0.0, GRB_CONTINUOUS,  "r_" + itos(j));
			R_j.push_back(r);
		}
		if(!aprx_tx_curve){
			// Create callback class
			callback_class callback_object = callback_class(&R_j, &Dn_j);
		}

		//
		/// Create constraints
		//

		// Track distance from waypoint-to-waypoint
		for(int i = 0, j = 1; j < M_k; i++, j++) {
			model.addQConstr(Di_j.at(j)*Di_j.at(j) >= (X_j.at(i)-X_j.at(j))*(X_j.at(i)-X_j.at(j)) + (Y_j.at(i)-Y_j.at(j))*(Y_j.at(i)-Y_j.at(j)) + (Z_j.at(i)-Z_j.at(j))*(Z_j.at(i)-Z_j.at(j)), "Di_"+itos(j)+"_geq_dist-i-j");
		}

		// Track distance from BS-to-waypoint
		model.addQConstr(Di_j.at(0)*Di_j.at(0) >= (X_j.at(0)-X_b)*(X_j.at(0)-X_b) + (Y_j.at(0)-Y_b)*(Y_j.at(0)-Y_b) + (Z_j.at(0)-Z_b)*(Z_j.at(0)-Z_b), "Db0_geq_dist-b-0");
		// Track distance from waypoint-to-BS
		model.addQConstr(Dnb*Dnb >= (X_j.at(M_k-1)-X_b)*(X_j.at(M_k-1)-X_b) + (Y_j.at(M_k-1)-Y_b)*(Y_j.at(M_k-1)-Y_b) + (Z_j.at(M_k-1)-Z_b)*(Z_j.at(M_k-1)-Z_b), "Dnb_geq_dist-n-b");

		// Track distance from node-to-waypoint
		for(int j = 0; j < M_k; j++) {
			model.addQConstr(Dn_j.at(j)*Dn_j.at(j) >= (X_j.at(j)-Xn_j.at(j))*(X_j.at(j)-Xn_j.at(j)) + (Y_j.at(j)-Yn_j.at(j))*(Y_j.at(j)-Yn_j.at(j)) + (Z_j.at(j)-Zn_j.at(j))*(Z_j.at(j)-Zn_j.at(j)), "Dn_"+itos(j)+"_geq_dist");
		}

		// Limit service time
		for(int j = 0; j < M_k; j++) {
			int i = sub_tour->at(j).node_id;
			model.addQConstr(Ts_j.at(j)*R_j.at(j) >= input->getQ_i(i), "R_"+itos(j)+"_geq_Q/Ts");
		}

		// Constrain energy consumption (based on time)
		{
			GRBLinExpr lhs = 0;

			// Add in energy to travel from i->j (NOTE: i->0 is bs->0)
			for(int j = 0; j < M_k; j++) {
				lhs += Di_j.at(j)*(1.0/input->getV_l(l))*input->getRho_m(l);
			}

			// Add in energy to travel from j->bs
			lhs += Dnb*(1.0/input->getV_l(l))*input->getRho_m(l);

			// Add in time to service node j
			for(int j = 0; j < M_k; j++) {
				lhs += Ts_j.at(j)*input->getRho_h(l);
			}

			model.addConstr(lhs <= input->getB_l(l), "pT_l_leq_b");
		}
		

		// Limit TX rate
		if(aprx_tx_curve) {
			// Use a PWL approximation
			for(int j = 0; j < M_k; j++) {
				// Get battery details for this node
				int node = sub_tour->at(j).node_id;
				double a, b, max_rate, c;
				input->getTXParams_i(node, &a, &b, &max_rate, &c);

				// Compute points (D, R) of R = a/(D)^2 + b for some step length
				double intv = 2.0;
				double xmax = 150.0;
				int len = (int) ceil((xmax-sqrt(a/(max_rate - b) - c))/intv) + 1;
				std::cout << "!!!Length " << len << std::endl;
				double* xpts = new double[len];
				double* upts = new double[len];
				xpts[0] = 0.0;
				upts[0] = max_rate;
				xpts[1] = sqrt(a/(max_rate - b) - c);
				upts[1] = max_rate;
				for(int i = 2; i < len; i++) {
					xpts[i] = i*intv + xpts[1];
					std::cout << "!!!xpts " << xpts[i] << std::endl;
					upts[i] = std::min(a/(pow(xpts[i], 2) + c) + b, max_rate);
					std::cout << "!!!upts " << upts[i] << std::endl;
				}
				std::cout << "!!!Parameters," << j << "," << a  << "," << b << "," << max_rate << "," << c << "\n";

				model.addGenConstrPWL(Dn_j.at(j), R_j.at(j), len, xpts, upts, "R_"+itos(j)+"_leq_math");
			} }
			else{
				// Single linear approximation using lookup table
				for(int j = 0; j < M_k; j++) {
					int node_id = sub_tour->at(j).node_id;
					// Get battery details for this node
					double a, b, max_rate, c;
					input->getTXParams_i(node_id, &a, &b, &max_rate, &c);
					// get node type (pi 3 or pi 4)
					double node_type;
					node_type = input->getNodeType_i(node_id);
					// get packet size for this node
					double q;
					q = input->getQ_i(node_id);
					// get velocity for this drone
					double velocity;
					velocity = input->getV_l(l);
					// Use lookup table to get distance
					double distance = input->lookup_distance(node_type, q, velocity);


					// Determine line equation to approximate TX rate curve
					// First two points are the saturated TX rate
					double y1 = max_rate;
					double x1 = sqrt(a/(y1-b)-c);
					// Second two points 
					// double x2 = sqrt(a/(y2-b)-c); // lookup
					double x2 = distance;
					// double y2 = max_rate/2.0;
					double y2 = a/(pow(x2, 2) + c) + b;
					// double m = (y2-y1)/(x2-x1);

					int len = 2;
					double* xpts = new double[len];
					double* upts = new double[len];

					xpts[0] = x1;
					xpts[1] = x2;
					upts[0] = y1;
					upts[1] = y2;

					double m = (y2-y1)/(x2-x1);


					// if(DEBUG_CV_OPTMZR)
						// printf(" %d : a=%.2f, b=%.2f, max_rate=%.2f, c=%.2f, m=%.2f, (x1,y1)=(%.2f,%.2f)\n", i, a, b, max_rate, c,m,x1,y1);

					model.addQConstr(R_j.at(j) <= (m*(Dn_j.at(j) - x2) + y2), "R_"+itos(j)+"_leq_math");
					// model.addGenConstrPWL(Dn_j.at(j), R_j.at(j), 2, xpts, upts, "R_"+itos(j)+"_leq_math");
				}
			}
			// else {
			// 	// Single linear approximation (fast!)
			// 	for(int j = 0; j < M_k; j++) {
			// 		// Get battery details for this node
			// 		int i = sub_tour->at(j).node_id;
			// 		double a, b, max_rate, c;
			// 		input->getTXParams_i(i, &a, &b, &max_rate, &c);

			// 		// Determine line equation to approximate TX rate curve
			// 		double y1 = max_rate;
			// 		double x1 = sqrt(a/(y1-b)-c);
			// 		double y2 = max_rate/2.0;
			// 		double x2 = sqrt(a/(y2-b)-c); // lookup
			// 		double m = (y2-y1)/(x2-x1);

			// 		if(DEBUG_CV_OPTMZR)
			// 			printf(" %d : a=%.2f, b=%.2f, max_rate=%.2f, c=%.2f, m=%.2f, (x1,y1)=(%.2f,%.2f)\n",i,a, b, max_rate, c,m,x1,y1);

			// 		model.addQConstr(R_j.at(j) <= m*(Dn_j.at(j) - x1) + y1, "R_"+itos(j)+"_leq_math");
			// 	}
			// }


		//
		/// Set the objective function
		//

		{
			GRBLinExpr objective = 0;

			// Add in time to travel from i->j (NOTE: i->0 is bs->0)
			for(int j = 0; j < M_k; j++) {
				objective += Di_j.at(j)*(1.0/input->getV_l(l));
			}

			// Add in time to travel from i->bs
			objective += Dnb*(1.0/input->getV_l(l));

			// Add in time to service node i
			for(int j = 0; j < M_k; j++) {
				objective += Ts_j.at(j);
			}

			model.setObjective(objective, GRB_MINIMIZE);
		}


		//
		/// Run optimizer
		//
	
		model.optimize();
		


		if(model.get(GRB_IntAttr_SolCount) >= 1) {
			//
			/// Print Solution
			//
			if(DEBUG_CV_OPTMZR) {
				printf("Obj: %f\n", model.get(GRB_DoubleAttr_ObjVal));

				// Each hovering location
				printf("Hovering Order and Locations:\n");
				for(int j = 0; j < M_k; j++) {
					int i = sub_tour->at(j).node_id;
					printf(" %d : (%.2f, %.2f, %.2f) - %.2fm -> (%.2f, %.2f, %.2f), %.2f @ %.2f Mb/s\n",
							j, input->getX_i(i), input->getY_i(i), input->getZ_i(i), Dn_j.at(j).get(GRB_DoubleAttr_X),
							X_j.at(j).get(GRB_DoubleAttr_X), Y_j.at(j).get(GRB_DoubleAttr_X), Z_j.at(j).get(GRB_DoubleAttr_X),
							Ts_j.at(j).get(GRB_DoubleAttr_X), R_j.at(j).get(GRB_DoubleAttr_X));
				}
			}

			// Save the solution into the sub-tour array
			for(int j = 0; j < M_k; j++) {
				sub_tour->at(j).x = X_j.at(j).get(GRB_DoubleAttr_X);
				sub_tour->at(j).y = Y_j.at(j).get(GRB_DoubleAttr_X);
				sub_tour->at(j).z = Z_j.at(j).get(GRB_DoubleAttr_X);
			}
		}
		else {
			return false;
		}

	} catch(GRBException e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	}  catch(const std::exception& e) {
		printf("Exception during optimization: %s\n", e.what());
	}

	return true;
}
