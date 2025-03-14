#include "Solver_Standard.h"
#include "Online_Input.h"


Solver_Standard::Solver_Standard(Constraint_tx_type constraint_type) : COptimizer(constraint_type) {
	if(SANITY_PRINT)
		printf("Hello from Standard Solver!\n");
}


void Solver_Standard::Solve(Input* input, Solution* I_crnt) {
	// Cast pointer to be input
	Online_Input* onlineInput = (Online_Input*)input;

	int n_k = onlineInput->getNk();
	const std::vector<int>* sub_tour = onlineInput->getNextList();
	int firstNode_i = onlineInput->getFirst_i();

	// Is there only 1 nodes left (i.e. is next-list empty)?
	if(sub_tour->size() == 0) {
		// Only 1 node left.. just visit that one
		HoveringLocation hl(onlineInput->getX_f(), onlineInput->getY_f(), onlineInput->getZ_f() + input->getZs_i(firstNode_i), firstNode_i);
		I_crnt->AddHL(hl);
	}
	else {
		// Multiple nodes...
		try {
			//
			/// Create an environment
			//
			GRBEnv env = GRBEnv(true);
			env.set("LogFile", "mip1.log");
			env.start();
			GRBModel model = GRBModel(env);

			// Are we using non-convex constraints for TX rate?
			if(constraint_type == Constraint_tx_type::PWL or constraint_type == Constraint_tx_type::INVERSE_SQUARE) {
				// Tell Gurobi this is non-convex
				model.set(GRB_IntParam_NonConvex, 2);
				model.set(GRB_DoubleParam_TimeLimit, 30.0);
			}

			if(DEBUG_SLVR_STD) {
				printf("Starting up Gurobi\n");
			}
			else {
				model.set(GRB_INT_PAR_OUTPUTFLAG, "0");
				model.set(GRB_INT_PAR_LOGTOCONSOLE, "0");
			}

			//
			/// Create variables
			//

			// Create position variables for each waypoint
			std::vector<GRBVar> X_j;
			std::vector<GRBVar> Y_j;
			std::vector<GRBVar> Z_j;

			for(int j = 0; j < n_k; j++) {
				int i = sub_tour->at(j);
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

			// Create position variables for first node (relaxed on node position)
			GRBVar Xn_f = model.addVar(CONST_RELAXATION(onlineInput->getX_f()), 0.0, GRB_CONTINUOUS, "xn_f");
			GRBVar Yn_f = model.addVar(CONST_RELAXATION(onlineInput->getY_f()), 0.0, GRB_CONTINUOUS, "yn_f");
			GRBVar Zn_f = model.addVar(CONST_RELAXATION(onlineInput->getZ_f()), 0.0, GRB_CONTINUOUS, "zn_f");

			// Create position variables for each node (relaxed on node position)
			std::vector<GRBVar> Xn_j;
			std::vector<GRBVar> Yn_j;
			std::vector<GRBVar> Zn_j;
			for(int j = 0; j < n_k; j++) {
				int i = sub_tour->at(j);
				GRBVar x = model.addVar(CONST_RELAXATION(input->getX_i(i)), 0.0, GRB_CONTINUOUS, "xn_" + itos(j));
				GRBVar y = model.addVar(CONST_RELAXATION(input->getY_i(i)), 0.0, GRB_CONTINUOUS, "yn_" + itos(j));
				GRBVar z = model.addVar(CONST_RELAXATION(input->getZ_i(i)), 0.0, GRB_CONTINUOUS, "zn_" + itos(j));

				Xn_j.push_back(x);
				Yn_j.push_back(y);
				Zn_j.push_back(z);
			}

			// Create position variables for our starting position
			GRBVar X_s = model.addVar(CONST_RELAXATION(onlineInput->getX_s()), 0.0, GRB_CONTINUOUS, "x_s");
			GRBVar Y_s = model.addVar(CONST_RELAXATION(onlineInput->getY_s()), 0.0, GRB_CONTINUOUS, "y_s");
			GRBVar Z_s = model.addVar(CONST_RELAXATION(onlineInput->getZ_s()), 0.0, GRB_CONTINUOUS, "z_s");

			// Create position variables for the first stop (a fixed location)
			GRBVar X_f = model.addVar(CONST_RELAXATION(onlineInput->getX_f()), 0.0, GRB_CONTINUOUS, "x_f");
			GRBVar Y_f = model.addVar(CONST_RELAXATION(onlineInput->getY_f()), 0.0, GRB_CONTINUOUS, "y_f");
			GRBVar Z_f = model.addVar(CONST_RELAXATION(onlineInput->getZ_f() + input->getZs_i(firstNode_i)), 0.0, GRB_CONTINUOUS, "z_f");

			// Create position variables for base station
			GRBVar X_b = model.addVar(CONST_RELAXATION(input->getX_b()), 0.0, GRB_CONTINUOUS, "x_b");
			GRBVar Y_b = model.addVar(CONST_RELAXATION(input->getY_b()), 0.0, GRB_CONTINUOUS, "y_b");
			GRBVar Z_b = model.addVar(CONST_RELAXATION(input->getZ_b()), 0.0, GRB_CONTINUOUS, "z_b");

			// Create distance i -> j variables
			std::vector<GRBVar> Di_j;
			for(int j = 0; j < n_k; j++) {
				// Distance from waypoint i to j
				GRBVar d = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "di_" + itos(j));
				Di_j.push_back(d);
			}

			// Create distance s -> f variable
			GRBVar Dsf = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "Dsf");
			// Create distance j -> bs variable
			GRBVar Dnb = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "Dnb");

			// Create distance node_f -> wp_f variable
			GRBVar Dn_f = model.addVar(0.1, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "dn_f");

			// Create distance node_i -> wp_i variables
			std::vector<GRBVar> Dn_j;
			for(int j = 0; j < n_k; j++) {
				// Distance from node i to waypoint i
				GRBVar d = model.addVar(0.1, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "dn_" + itos(j));
				Dn_j.push_back(d);
			}

			// Create service time variable for first node
			GRBVar Ts_f = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "ts_f");

			// Create service time variables
			std::vector<GRBVar> Ts_j;
			for(int j = 0; j < n_k; j++) {
				// Sequence number for waypoint i
				GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "ts_" + itos(j));
				Ts_j.push_back(t);
			}

			// Create TX rate variable for first node
			GRBVar R_f;
			{
				double a, b, maxR, C, minR;
				input->getTXParams_i(firstNode_i, &a, &b, &maxR, &C, &minR);
				R_f = model.addVar(minR, maxR, 0.0, GRB_CONTINUOUS,  "r_");
			}

			// Create TX rate variables
			std::vector<GRBVar> R_j;
			for(int j = 0; j < n_k; j++) {
				int i = sub_tour->at(j);
				// Get battery details
				double a, b, maxR, C, minR;
				input->getTXParams_i(i, &a, &b, &maxR, &C, &minR);
				// Sequence number for waypoint i
				GRBVar r = model.addVar(minR, maxR, 0.0, GRB_CONTINUOUS,  "r_" + itos(j));
				R_j.push_back(r);
			}


			//
			/// Create constraints
			//

			// Track distance from waypoint-to-waypoint
			for(int i = 0, j = 1; j < n_k; i++, j++) {
				model.addQConstr(Di_j.at(j)*Di_j.at(j) >= (X_j.at(i)-X_j.at(j))*(X_j.at(i)-X_j.at(j)) + (Y_j.at(i)-Y_j.at(j))*(Y_j.at(i)-Y_j.at(j)) + (Z_j.at(i)-Z_j.at(j))*(Z_j.at(i)-Z_j.at(j)), "Di_"+itos(j)+"_geq_dist-i-j");
			}

			// Track distance from start-to-first
			model.addQConstr(Dsf*Dsf >= (X_s-X_f)*(X_s-X_f) + (Y_s-Y_f)*(Y_s-Y_f) + (Z_s-Z_f)*(Z_s-Z_f), "Dsf_geq_dist-s-f");

			// Track distance from first-to-waypoint1
			model.addQConstr(Di_j.at(0)*Di_j.at(0) >= (X_j.at(0)-X_f)*(X_j.at(0)-X_f) + (Y_j.at(0)-Y_f)*(Y_j.at(0)-Y_f) + (Z_j.at(0)-Z_f)*(Z_j.at(0)-Z_f), "Df0_geq_dist-f-0");

			// Track distance from waypoint-to-BS
			model.addQConstr(Dnb*Dnb >= (X_j.at(n_k-1)-X_b)*(X_j.at(n_k-1)-X_b) + (Y_j.at(n_k-1)-Y_b)*(Y_j.at(n_k-1)-Y_b) + (Z_j.at(n_k-1)-Z_b)*(Z_j.at(n_k-1)-Z_b), "Dnb_geq_dist-n-b");

			// Track distance from node-to-waypoint
			for(int j = 0; j < n_k; j++) {
				model.addQConstr(Dn_j.at(j)*Dn_j.at(j) >= (X_j.at(j)-Xn_j.at(j))*(X_j.at(j)-Xn_j.at(j)) + (Y_j.at(j)-Yn_j.at(j))*(Y_j.at(j)-Yn_j.at(j)) + (Z_j.at(j)-Zn_j.at(j))*(Z_j.at(j)-Zn_j.at(j)), "Dn_"+itos(j)+"_geq_dist");
			}
			// Distance from first_node-to-first_waypoint

			model.addQConstr(Dn_f*Dn_f >= (X_f-Xn_f)*(X_f-Xn_f) + (Y_f-Yn_f)*(Y_f-Yn_f) + (Z_f-Zn_f)*(Z_f-Zn_f), "Dn_f_geq_dist");

			// Limit service time for first node  Ts_f
			model.addQConstr(Ts_f*R_f >= input->getQ_i(firstNode_i), "R_f_geq_Q/Ts");

			// Limit service time
			for(int j = 0; j < n_k; j++) {
				int i = sub_tour->at(j);
				model.addQConstr(Ts_j.at(j)*R_j.at(j) >= input->getQ_i(i), "R_"+itos(j)+"_geq_Q/Ts");
			}

			// Constrain energy consumption (based on time)
			{
				GRBLinExpr lhs = 0;

				// Add in energy to travel from s->f
				lhs += Dsf*(1.0/onlineInput->getV())*onlineInput->getRho_m();

				// Add in energy to travel from i->j (NOTE: i->0 is bs->0)
				for(int j = 0; j < n_k; j++) {
					lhs += Di_j.at(j)*(1.0/onlineInput->getV())*onlineInput->getRho_m();
				}

				// Add in energy to travel from j->bs
				lhs += Dnb*(1.0/onlineInput->getV())*onlineInput->getRho_m();

				// Add in time to service first node
				lhs += Ts_f*onlineInput->getRho_h();

				// Add in time to service node j
				for(int j = 0; j < n_k; j++) {
					lhs += Ts_j.at(j)*onlineInput->getRho_h();
				}

				model.addConstr(lhs <= onlineInput->getB(), "pT_l_leq_b");
			}

			// Limit TX rate for first node (set to rate when directly above node)
			{
				// Get battery details for this node
				double a, b, maxR, c, minR;
				input->getTXParams_i(firstNode_i, &a, &b, &maxR, &c, &minR);

				// Determine line equation to approximate TX rate curve
				double z_s = input->getZs_i(firstNode_i);
				double rate = a/(z_s*z_s+c)+b;
				rate = rate > maxR ? maxR : rate;

				model.addQConstr(R_f <= rate + EPSILON, "Rf_leq_math");
			}

			std::vector<Point> sub_tour_points;
			for(int j = 0; j < n_k; j++) {
				// Each node that is left
				int i = sub_tour->at(j);
				Point temp(i, input->getX_i(i), input->getY_i(i), input->getZ_i(i));
				sub_tour_points.push_back(temp);
			}

			// Limit TX rate
			switch (constraint_type)
			{
			case Constraint_tx_type::TABULAR_CUT:
				GenerateLazyConstraint(0, model, &sub_tour_points, input, &R_j, &Dn_j);
				break;

			case Constraint_tx_type::SINGLE_APPROXIMATION:
				GenerateSingleApproxConstraint(model, &sub_tour_points, input, &R_j, &Dn_j);
				break;

			default:
				std::cout << "Unrecognized constraint tx type" << std::endl;
				break;
			}



			//
			/// Set the objective function
			//

			{
				GRBLinExpr objective = 0;

				// Add in energy to travel from s->f
				objective += Dsf*(1.0/onlineInput->getV());

				// Add in time to travel from i->j (NOTE: i->0 is bs->0)
				for(int j = 0; j < n_k; j++) {
					objective += Di_j.at(j)*(1.0/onlineInput->getV());
				}

				// Add in time to travel from i->bs
				objective += Dnb*(1.0/onlineInput->getV());

				// Add in time to service node i
				objective += Ts_f;

				// Add in time to service node i
				for(int j = 0; j < n_k; j++) {
					objective += Ts_j.at(j);
				}

				model.setObjective(objective, GRB_MINIMIZE);
			}


			//
			/// Run optimizer
			//
			model.optimize();


			//
			/// Print Solution
			//
			if(DEBUG_SLVR_STD) {
				printf("Obj: %f\n", model.get(GRB_DoubleAttr_ObjVal));

				// Each hovering location
				printf("Hovering Order and Locations:\n");
				printf(" f : (%.2f, %.2f, %.2f) - %.2fm -> (%.2f, %.2f, %.2f), %.2f @ %.2f Mb/s\n",
						onlineInput->getX_f(), onlineInput->getY_f(), onlineInput->getZ_f(), Dn_f.get(GRB_DoubleAttr_X),
						X_f.get(GRB_DoubleAttr_X), Y_f.get(GRB_DoubleAttr_X), Z_f.get(GRB_DoubleAttr_X),
						Ts_f.get(GRB_DoubleAttr_X), R_f.get(GRB_DoubleAttr_X));
				for(int j = 0; j < n_k; j++) {
					int i = sub_tour->at(j);
					printf(" %d : (%.2f, %.2f, %.2f) - %.2fm -> (%.2f, %.2f, %.2f), %.2f @ %.2f Mb/s\n",
							j, input->getX_i(i), input->getY_i(i), input->getZ_i(i), Dn_j.at(j).get(GRB_DoubleAttr_X),
							X_j.at(j).get(GRB_DoubleAttr_X), Y_j.at(j).get(GRB_DoubleAttr_X), Z_j.at(j).get(GRB_DoubleAttr_X),
							Ts_j.at(j).get(GRB_DoubleAttr_X), R_j.at(j).get(GRB_DoubleAttr_X));
				}
			}

			// Save the solution into the cords array
			HoveringLocation hl(X_f.get(GRB_DoubleAttr_X), Y_f.get(GRB_DoubleAttr_X), Z_f.get(GRB_DoubleAttr_X), firstNode_i);
			I_crnt->AddHL(hl);
			for(int j = 0; j < n_k; j++) {
				HoveringLocation hl(X_j.at(j).get(GRB_DoubleAttr_X), Y_j.at(j).get(GRB_DoubleAttr_X), Z_j.at(j).get(GRB_DoubleAttr_X), sub_tour->at(j));
				I_crnt->AddHL(hl);
			}

		} catch(GRBException e) {
			std::cout << "Error code = " << e.getErrorCode() << std::endl;
			std::cout << e.getMessage() << std::endl;
		}  catch(const std::exception& e) {
			printf("Exception during optimization: %s\n", e.what());
		}
	}

	return;
}



