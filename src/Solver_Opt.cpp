#include "Solver_Opt.h"


Solver_Opt::Solver_Opt() {
	if(SANITY_PRINT)
		printf("Hello from Solver_Opt!\n");
	srand (time(NULL));
}


void Solver_Opt::Solve(Input* input, Solution* I_crnt) {
	try {
		//
		/// Create an environment
		//
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", "mip1.log");
		env.start();
		GRBModel model = GRBModel(env);
		model.set(GRB_IntParam_NonConvex, 2);

		//
		/// Create variables
		//
		// Create position variables for each node
		std::vector<GRBVar> X_i;
		std::vector<GRBVar> Y_i;
		for(int i = 0; i < input->getN(); i++) {
			GRBVar x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + itos(i));
			GRBVar y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + itos(i));

			X_i.push_back(x);
			Y_i.push_back(y);
		}

		// Create i->j sub-tour visit flags
		std::vector<std::vector<std::vector<std::vector<GRBVar>>>> Z_lijk;
		// For each drone...
		for(int l = 0; l < input->getM(); l++) {
			// Create node i dimension
			std::vector<std::vector<std::vector<GRBVar>>> node_i;
			for(int i = 0; i < input->getN(); i++) {
				// Create node j dimension
				std::vector<std::vector<GRBVar>> node_j;
				for(int j = 0; j < input->getN(); j++) {
					// Create sub-tour dimension
					std::vector<GRBVar> sub_tour;
					for(int k = 0; k < input->getN(); k++) {
						// Are we going in circles?
						if(i == j) {
							// Yes, keep this variable false
							GRBVar z = model.addVar(0.0, 0.0, 0.0, GRB_BINARY, "z_" + itos(l) + itos(i) + itos(j) + itos(k));
							sub_tour.push_back(z);
						}
						else {
							// No, i -> j is a valid transition
							// Does drone l travel from waypoint i to j on sub-tour k?
							GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z_" + itos(l) + itos(i) + itos(j) + itos(k));
							sub_tour.push_back(z);
						}
					}
					// Push sub-tour dimension
					node_j.push_back(sub_tour);
				}
				// Push node j dimension
				node_i.push_back(node_j);
			}
			// Push node i dimension
			Z_lijk.push_back(node_i);
		}

		// Create sub-tour activation flags
		std::vector<std::vector<GRBVar>> U_lk;
		// For each drone
		for(int l = 0; l < input->getM(); l++) {
			// For each sub-tour
			std::vector<GRBVar> sub_tour;
			for(int k = 0; k < input->getN(); k++) {
				// Does drone l use sub-tour k?
				GRBVar u = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "u_" + itos(l) + itos(k));
				sub_tour.push_back(u);
			}
			// Push sub-tour
			U_lk.push_back(sub_tour);
		}

		// Create energy budget variables
		std::vector<std::vector<GRBVar>> B_lk;
		// For each drone
		for(int l = 0; l < input->getM(); l++) {
			// For each sub-tour
			std::vector<GRBVar> sub_tour;
			for(int k = 0; k < input->getN(); k++) {
				// Energy spent by l on sub-tour k
				GRBVar b = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "b_" + itos(l) + itos(k));
				sub_tour.push_back(b);
			}
			// Push sub-tour
			B_lk.push_back(sub_tour);
		}

		// Create sub-tour time duration variables
		std::vector<std::vector<GRBVar>> T_lk;
		// For each drone
		for(int l = 0; l < input->getM(); l++) {
			// For each sub-tour
			std::vector<GRBVar> sub_tour;
			for(int k = 0; k < input->getN(); k++) {
				// Completion time of sub-tour k
				GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "t_" + itos(l) + itos(k));
				sub_tour.push_back(t);
			}
			// Push sub-tour
			T_lk.push_back(sub_tour);
		}

		// Create distance i -> j variables
		std::vector<std::vector<GRBVar>> D_ij;
		for(int i = 0; i < input->getN(); i++) {
			// Create node j dimension
			std::vector<GRBVar> node_j;
			for(int j = 0; j < input->getN(); j++) {
				// Distance from waypoint i to j
				GRBVar d = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "d_" + itos(i) + itos(j));
				node_j.push_back(d);
			}
			// Push node j dimension
			D_ij.push_back(node_j);
		}

		// Create waypoint/node sequence variable
		std::vector<GRBVar> W_i;
		for(int i = 0; i < input->getN(); i++) {
			// Sequence number for waypoint i
			GRBVar w = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,  "w_" + itos(i));
			W_i.push_back(w);
		}

		// Create service time variable
		std::vector<GRBVar> Ts_i;
		for(int i = 0; i < input->getN(); i++) {
			// Sequence number for waypoint i
			GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER,  "ts_" + itos(i));
			Ts_i.push_back(t);
		}

		//
		/// Create constraints
		//

		// Constrain each sub-tour time
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				GRBQuadExpr lhs = T_lk.at(l).at(k);

				// Add in time to travel from i->j (if traveling from i->j)
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						lhs = lhs - D_ij.at(i).at(j)*Z_lijk.at(l).at(i).at(j).at(k)*(1.0/input->getV_l(l));
					}
				}

				// Add in time to service node i
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						lhs = lhs - Ts_i.at(i)*Z_lijk.at(l).at(j).at(i).at(k);
					}
				}

				// The first sub-tour is free... the rest need a battery swap
				if(k > 0) {
					lhs = lhs - input->getTb_l(l)*U_lk.at(l).at(k);
				}

				model.addQConstr(lhs >= 0, "T_"+itos(l)+itos(k)+"_geq_tour_time");
			}
		}

		// Track distance from waypoint-to-waypoint
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getN(); j++) {
				model.addQConstr(D_ij.at(i).at(j)*D_ij.at(i).at(j) >= (X_i.at(i)-X_i.at(j))*(X_i.at(i)-X_i.at(j)) + (Y_i.at(i)-Y_i.at(j)), "D_"+itos(i)+itos(j)+"_geq_dist");
			}
		}

		// Visit all waypoints
		for(int j = 0; j < input->getN(); j++) {
			GRBLinExpr expr = 0;
			for(int l = 0; l < input->getM(); l++) {
				for(int i = 0; i < input->getN(); i++) {
					for(int k = 0; k < input->getN(); k++) {
						expr = expr + Z_lijk.at(l).at(i).at(j).at(k);
					}
				}
			}

			model.addConstr(expr == 1, "Z_li"+itos(j)+"_eq_1");
		}

		// Control sub-tour usage
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				// If not U then..
				GRBLinExpr expr = BIG_M*U_lk.at(l).at(k);

				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						expr -= Z_lijk.at(l).at(i).at(j).at(k);
					}
				}

				// All Z should be turned off
				model.addConstr(expr >= 0, "MU_"+itos(l)+itos(k)+"_eq_Z");
			}
		}

		// Current law
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				for(int i = 0; i < input->getN(); i++) {
					GRBLinExpr expr = 0;
					// Sum going in...
					for(int j = 0; j < input->getN(); j++) {
						expr += Z_lijk.at(l).at(j).at(i).at(k);
					}
					// Minus sum going out...
					for(int j = 0; j < input->getN(); j++) {
						expr -= Z_lijk.at(l).at(i).at(j).at(k);
					}


					// Equals zero
					model.addConstr(expr == 0, "Z_"+itos(l)+itos(i)+"j"+itos(k)+"_eq_Z_"+itos(l)+"j"+itos(i)+itos(k));
				}
			}
		}

		// Enforce the numbering system
		for(int l = 0; l < input->getM(); l++) {
			for(int i = 0; i < input->getN(); i++) {
				for(int j = 0; j < input->getN(); j++) {
					for(int k = 0; k < input->getN(); k++) {
						// LHS bound
						model.addConstr(BIG_M*(1-Z_lijk.at(l).at(i).at(j).at(k)) + W_i.at(i) + 1 >= W_i.at(j), "if_Z_"+itos(l)+itos(i)+itos(j)+itos(k)+"_then_W");
						// RHS bound
						model.addConstr( W_i.at(j) >= W_i.at(i) + 1 - BIG_M*(1 - Z_lijk.at(l).at(i).at(j).at(k)), "then_W_if_Z_"+itos(l)+itos(i)+itos(j)+itos(k));
					}
				}
			}
		}

		// Control the numbering scheme to start with the base station
		// Control time required to collect data
		// Restrain distance from node...?



		//
		/// Set the objective function
		//
		GRBLinExpr objective = 0;
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				objective += T_lk.at(l).at(k);
			}
		}
		model.setObjective(objective, GRB_MINIMIZE);

		//
		/// Run optimizer
		//
		model.optimize();









//		// Create variables
//		GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
//		GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
//		GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");
//
//		// Set objective: maximize x + y + 2 z
//		model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);
//
//		// Add constraint: x + 2 y + 3 z <= 4
//		model.addConstr(x + 2 * y + 3 * z <= 4, "c0");
//
//		// Add constraint: x + y >= 1
//		model.addConstr(x + y >= 1, "c1");
//
//		// Optimize model
//		model.optimize();
//
//		std::cout << x.get(GRB_StringAttr_VarName) << " "
//			 << x.get(GRB_DoubleAttr_X) << std::endl;
//		std::cout << y.get(GRB_StringAttr_VarName) << " "
//			 << y.get(GRB_DoubleAttr_X) << std::endl;
//		std::cout << z.get(GRB_StringAttr_VarName) << " "
//			 << z.get(GRB_DoubleAttr_X) << std::endl;


		//
		/// Print Solution
		//
		if(DEBUG_SLVR_OPT) {
			printf("Obj: %f\n", model.get(GRB_DoubleAttr_ObjVal));

			// Each hovering location
			printf("Hovering Order and Locations:\n");
			for(int i = 0; i < input->getN(); i++) {
				printf(" %d : %.1f (%f, %f)\n", i, W_i.at(i).get(GRB_DoubleAttr_X), X_i.at(i).get(GRB_DoubleAttr_X), Y_i.at(i).get(GRB_DoubleAttr_X));
			}

			// Tour variables...
			printf("Tour:\n");
			for(int l = 0; l < input->getM(); l++) {
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						for(int k = 0; k < input->getN(); k++) {
							if(Z_lijk.at(l).at(i).at(j).at(k).get(GRB_DoubleAttr_X) > 0.5) {
								printf(" %d goes %d->%d on %d\n", l, i, j, k);
							}
						}
					}
				}
			}
		}

	} catch(GRBException e) {
		std::cout << "Error code = " << e.getErrorCode() << std::endl;
		std::cout << e.getMessage() << std::endl;
	}  catch(const std::exception& e) {
		printf("Exception during optimization: %s\n", e.what());
	}

}
