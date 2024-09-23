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
		model.set(GRB_DoubleParam_TimeLimit, 20.0);

		//
		/// Create variables
		//

		// Create position variables for each waypoint
		std::vector<GRBVar> X_i;
		std::vector<GRBVar> Y_i;
		std::vector<GRBVar> Z_i;
		for(int i = 0; i < input->getN(); i++) {
			GRBVar x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + itos(i));
			GRBVar y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + itos(i));
			GRBVar z = model.addVar(input->getZ_i(i) + input->getZs_i(i), GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_" + itos(i));
			x.set(GRB_DoubleAttr_Start, input->getX_i(i));
			y.set(GRB_DoubleAttr_Start, input->getY_i(i));
			z.set(GRB_DoubleAttr_Start, input->getZ_i(i) + input->getZs_i(i));

			X_i.push_back(x);
			Y_i.push_back(y);
			Z_i.push_back(z);
		}

		// Create position variables for each node (relaxed on node position)
		std::vector<GRBVar> Xn_i;
		std::vector<GRBVar> Yn_i;
		std::vector<GRBVar> Zn_i;
		for(int i = 0; i < input->getN(); i++) {
			GRBVar x = model.addVar(CONST_RELAXATION(input->getX_i(i)), 0.0, GRB_CONTINUOUS, "xn_" + itos(i));
			GRBVar y = model.addVar(CONST_RELAXATION(input->getY_i(i)), 0.0, GRB_CONTINUOUS, "yn_" + itos(i));
			GRBVar z = model.addVar(CONST_RELAXATION(input->getZ_i(i)), 0.0, GRB_CONTINUOUS, "zn_" + itos(i));

			Xn_i.push_back(x);
			Yn_i.push_back(y);
			Zn_i.push_back(z);
		}

		// Create position variables for base station
		GRBVar X_b = model.addVar(CONST_RELAXATION(input->getX_b()), 0.0, GRB_CONTINUOUS, "x_b");
		GRBVar Y_b = model.addVar(CONST_RELAXATION(input->getY_b()), 0.0, GRB_CONTINUOUS, "y_b");
		GRBVar Z_b = model.addVar(CONST_RELAXATION(input->getZ_b()), 0.0, GRB_CONTINUOUS, "z_b");

		// Create i->j sub-tour visit flags
		std::vector<std::vector<std::vector<std::vector<GRBVar>>>> E_lijk;
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
			E_lijk.push_back(node_i);
		}

		// Create BS->i sub-tour visit flags
		std::vector<std::vector<std::vector<GRBVar>>> Eb_lik;
		// For each drone...
		for(int l = 0; l < input->getM(); l++) {
			// Create node i dimension
			std::vector<std::vector<GRBVar>> node_i;
			for(int i = 0; i < input->getN(); i++) {
				// Create sub-tour dimension
				std::vector<GRBVar> sub_tour;
				for(int k = 0; k < input->getN(); k++) {
					// Does drone l travel from BS to waypoint i on sub-tour k?
					GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "zb_" + itos(l) + itos(i) + itos(k));
					sub_tour.push_back(z);
				}
				// Push sub-tour dimension
				node_i.push_back(sub_tour);
			}
			// Push node i dimension
			Eb_lik.push_back(node_i);
		}

		// Create i->BS sub-tour visit flags
		std::vector<std::vector<std::vector<GRBVar>>> E_lik_b;
		// For each drone...
		for(int l = 0; l < input->getM(); l++) {
			// Create node i dimension
			std::vector<std::vector<GRBVar>> node_i;
			for(int i = 0; i < input->getN(); i++) {
				// Create sub-tour dimension
				std::vector<GRBVar> sub_tour;
				for(int k = 0; k < input->getN(); k++) {
					// Does drone l travel from BS to waypoint i on sub-tour k?
					GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z_" + itos(l) + itos(i) + itos(k) + "_b");
					sub_tour.push_back(z);
				}
				// Push sub-tour dimension
				node_i.push_back(sub_tour);
			}
			// Push node i dimension
			E_lik_b.push_back(node_i);
		}

		// Create sub-tour activation flags
		std::vector<std::vector<GRBVar>> W_lk;
		// For each drone
		for(int l = 0; l < input->getM(); l++) {
			// For each sub-tour
			std::vector<GRBVar> sub_tour;
			for(int k = 0; k < input->getN(); k++) {
				// Does drone l use sub-tour k?
				GRBVar u = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "u_" + itos(l) + itos(k));

				if(k > 0) {
					u.set(GRB_DoubleAttr_Start, 0);
				}
				else {
					u.set(GRB_DoubleAttr_Start, 1);
				}

				sub_tour.push_back(u);
			}
			// Push sub-tour
			W_lk.push_back(sub_tour);
		}

//		// Create energy budget variables
//		std::vector<std::vector<GRBVar>> B_lk;
//		// For each drone
//		for(int l = 0; l < input->getM(); l++) {
//			// For each sub-tour
//			std::vector<GRBVar> sub_tour;
//			for(int k = 0; k < input->getN(); k++) {
//				// Energy spent by l on sub-tour k
//				GRBVar b = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "b_" + itos(l) + itos(k));
//				sub_tour.push_back(b);
//			}
//			// Push sub-tour
//			B_lk.push_back(sub_tour);
//		}

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

		// Create distance bs ->i variables
		std::vector<GRBVar> D_bi;
		for(int i = 0; i < input->getN(); i++) {
			// Distance from BS to waypoint i
			GRBVar d = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "db_" + itos(i));
			D_bi.push_back(d);
		}

		// Create distance node_i -> wp_i variables
		std::vector<GRBVar> Dn_i;
		for(int i = 0; i < input->getN(); i++) {
			// Distance from node i to waypoint i
			GRBVar d = model.addVar(0.1, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "dn_" + itos(i));
			Dn_i.push_back(d);
		}

		// Create extra dummy distance node_i -> wp_i variables
		std::vector<GRBVar> Dnd_i;
		for(int i = 0; i < input->getN(); i++) {
			// Distance from node i to waypoint i
			GRBVar d = model.addVar(0.1, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "dd_" + itos(i));
			Dnd_i.push_back(d);
		}

		// Create waypoint/node sequence variable
		std::vector<GRBVar> U_i;
		for(int i = 0; i < input->getN(); i++) {
			// Sequence number for waypoint i
			GRBVar w = model.addVar(0.0, input->getN(), 0.0, GRB_INTEGER,  "w_" + itos(i));
			U_i.push_back(w);
		}

		// Create service time variable
		std::vector<GRBVar> Ts_i;
		for(int i = 0; i < input->getN(); i++) {
			// Sequence number for waypoint i
			GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,  "ts_" + itos(i));
			Ts_i.push_back(t);
		}

		// Create TX rate variable
		std::vector<GRBVar> R_i;
		for(int i = 0; i < input->getN(); i++) {
			// Get battery details
			double a, b, mrate;
			input->getTXParams_i(i, &a, &b, &mrate);
			// Sequence number for waypoint i
			GRBVar r = model.addVar(0.0, mrate, 0.0, GRB_CONTINUOUS,  "r_" + itos(i));
			R_i.push_back(r);
		}


		//
		/// Create constraints
		//

		// Constrain each sub-tour time
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				GRBQuadExpr rhs = 0;

				// Add in time to travel from i->j (if traveling from i->j)
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						rhs += D_ij.at(i).at(j)*E_lijk.at(l).at(i).at(j).at(k)*(1.0/input->getV_l(l));
					}
				}

				// Add in time to travel from bs->i (if traveling from bs->i)
				for(int i = 0; i < input->getN(); i++) {
					rhs += D_bi.at(i)*Eb_lik.at(l).at(i).at(k)*(1.0/input->getV_l(l));
				}

				// Add in time to travel from i->bs (if traveling from bs->i)
				for(int i = 0; i < input->getN(); i++) {
					rhs += D_bi.at(i)*E_lik_b.at(l).at(i).at(k)*(1.0/input->getV_l(l));
				}

				// Add in time to service node i
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						rhs += Ts_i.at(i)*E_lijk.at(l).at(j).at(i).at(k);
					}
				}

				// The first sub-tour is free... the rest need a battery swap
				if(k > 0) {
					rhs += input->getTb_l(l)*W_lk.at(l).at(k);
				}

				model.addQConstr(T_lk.at(l).at(k) >= rhs, "T_"+itos(l)+itos(k)+"_geq_tour_time");
			}
		}

		// Track distance from waypoint-to-waypoint
		for(int i = 0; i < input->getN(); i++) {
			for(int j = 0; j < input->getN(); j++) {
				model.addQConstr(D_ij.at(i).at(j)*D_ij.at(i).at(j) >= (X_i.at(i)-X_i.at(j))*(X_i.at(i)-X_i.at(j)) + (Y_i.at(i)-Y_i.at(j))*(Y_i.at(i)-Y_i.at(j)) + (Z_i.at(i)-Z_i.at(j))*(Z_i.at(i)-Z_i.at(j)), "D_"+itos(i)+itos(j)+"_geq_dist");
			}
		}

		// Control sub-tour usage
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				// If not U then..
				GRBLinExpr expr = BIG_M*W_lk.at(l).at(k);

				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						expr -= E_lijk.at(l).at(i).at(j).at(k);
					}
				}

				// All Z should be turned off
				model.addConstr(expr >= 0, "MU_"+itos(l)+itos(k)+"_eq_Z");
			}
		}

		// Enforce the numbering system
		for(int l = 0; l < input->getM(); l++) {
			for(int i = 0; i < input->getN(); i++) {
				for(int j = 0; j < input->getN(); j++) {
					for(int k = 0; k < input->getN(); k++) {
						// LHS bound
						GRBLinExpr lh_expr = BIG_M*(1-E_lijk.at(l).at(i).at(j).at(k)) + U_i.at(i) + 1 - U_i.at(j);
						model.addConstr(lh_expr >= 0, "if_Z_"+itos(l)+itos(i)+itos(j)+itos(k)+"_then_W");
						// RHS bound
						GRBLinExpr rh_expr = U_i.at(j) - U_i.at(i) - 1 + BIG_M*(1 - E_lijk.at(l).at(i).at(j).at(k));
						model.addConstr(rh_expr >= 0, "then_W_if_Z_"+itos(l)+itos(i)+itos(j)+itos(k));
					}
				}
			}
		}

		// Track distance from BS-to-waypoint
		for(int i = 0; i < input->getN(); i++) {
			model.addQConstr(D_bi.at(i)*D_bi.at(i) >= (X_i.at(i)-X_b)*(X_i.at(i)-X_b) + (Y_i.at(i)-Y_b)*(Y_i.at(i)-Y_b) + (Z_i.at(i)-Z_b)*(Z_i.at(i)-Z_b), "D_b"+itos(i)+"_geq_dist");
		}

		// If we use sub-tour k, then we have to leave the BS
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				GRBLinExpr expr = 0;
				for(int i = 0; i < input->getN(); i++) {
					// Sum of ways to leave...
					expr += Eb_lik.at(l).at(i).at(k);
				}
				// Equals sub-tour flag
				model.addConstr(expr == W_lk.at(l).at(k), "Zb_"+itos(l)+"i"+itos(k)+"_eq_W_"+itos(l)+itos(k));
			}
		}

		// If we use sub-tour k, then we have to return to the BS
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				GRBLinExpr expr = 0;
				for(int i = 0; i < input->getN(); i++) {
					// Sum of ways to return...
					expr += E_lik_b.at(l).at(i).at(k);
				}
				// Equals sub-tour flag
				model.addConstr(expr == W_lk.at(l).at(k), "Z_"+itos(l)+"i"+itos(k)+"_b_eq_W_"+itos(l)+itos(k));
			}
		}

		// Must "enter" every waypoint
		for(int i = 0; i < input->getN(); i++) {
			GRBLinExpr expr = 0;

			for(int l = 0; l < input->getM(); l++) {
				for(int j = 0; j < input->getN(); j++) {
					for(int k = 0; k < input->getN(); k++) {
						expr += E_lijk.at(l).at(j).at(i).at(k);
					}
				}
			}

			for(int l = 0; l < input->getM(); l++) {
				for(int k = 0; k < input->getN(); k++) {
					expr += Eb_lik.at(l).at(i).at(k);
				}
			}

			model.addConstr(expr == 1, "Sum_into_"+itos(i)+"_eq_1");
		}

		// Must "exit" every waypoint
		for(int i = 0; i < input->getN(); i++) {
			GRBLinExpr expr = 0;

			for(int l = 0; l < input->getM(); l++) {
				for(int j = 0; j < input->getN(); j++) {
					for(int k = 0; k < input->getN(); k++) {
						expr += E_lijk.at(l).at(i).at(j).at(k);
					}
				}
			}

			for(int l = 0; l < input->getM(); l++) {
				for(int k = 0; k < input->getN(); k++) {
					expr += E_lik_b.at(l).at(i).at(k);
				}
			}

			model.addConstr(expr == 1, "Sum_into_"+itos(i)+"_eq_1");
		}

		// Sum into waypoint == sum leaving
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				for(int i = 0; i < input->getN(); i++) {
					GRBLinExpr expr = 0;
					// Count going in
					for(int j = 0; j < input->getN(); j++) {
						expr += E_lijk.at(l).at(j).at(i).at(k);
					}
					expr += Eb_lik.at(l).at(i).at(k);

					// Minus count going out
					for(int j = 0; j < input->getN(); j++) {
						expr -= E_lijk.at(l).at(i).at(j).at(k);
					}
					expr -= E_lik_b.at(l).at(i).at(k);

					// Equals zero
					model.addConstr(expr == 0, "Sum_in_eq_sum_out_"+itos(i));
				}
			}
		}

		// Track distance from node-to-waypoint
		for(int i = 0; i < input->getN(); i++) {
			model.addQConstr(Dn_i.at(i)*Dn_i.at(i) >= (X_i.at(i)-Xn_i.at(i))*(X_i.at(i)-Xn_i.at(i)) + (Y_i.at(i)-Yn_i.at(i))*(Y_i.at(i)-Yn_i.at(i)) + (Z_i.at(i)-Zn_i.at(i))*(Z_i.at(i)-Zn_i.at(i)), "Dn_"+itos(i)+"_geq_dist");
		}

		// Limit TX rate
		for(int i = 0; i < input->getN(); i++) {
			// Get battery details
			double a, b, mrate;
			input->getTXParams_i(i, &a, &b, &mrate);
			model.addQConstr(R_i.at(i)*Dnd_i.at(i) - b*Dnd_i.at(i) <= a, "R_"+itos(i)+"_leq_math");
		}

		// Limit service time
		for(int i = 0; i < input->getN(); i++) {
			model.addQConstr(Ts_i.at(i)*R_i.at(i) >= input->getQ_i(i), "R_"+itos(i)+"_geq_Q/Ts");
		}

		// Track distance from node-to-waypoint
		for(int i = 0; i < input->getN(); i++) {
			model.addQConstr(Dnd_i.at(i) - Dn_i.at(i)*Dn_i.at(i) >= 0, "Dnd_"+itos(i)+"_geq_Dn");
		}

		// Order sub-tours
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN()-1; k++) {
				model.addConstr(W_lk.at(l).at(k) >= W_lk.at(l).at(k+1), "W_"+itos(l)+itos(k)+"_geq_W+1");
			}
		}

		// Constrain energy consumption (based on time)
		for(int l = 0; l < input->getM(); l++) {
			for(int k = 0; k < input->getN(); k++) {
				GRBQuadExpr lhs = 0;

				// Add in energy to travel from i->j (if traveling from i->j)
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						lhs += D_ij.at(i).at(j)*E_lijk.at(l).at(i).at(j).at(k)*(1.0/input->getV_l(l))*input->getRho_m(l);
					}
				}

				// Add in energy to travel from bs->i (if traveling from bs->i)
				for(int i = 0; i < input->getN(); i++) {
					lhs += D_bi.at(i)*Eb_lik.at(l).at(i).at(k)*(1.0/input->getV_l(l))*input->getRho_m(l);
				}

				// Add in energy to travel from i->bs (if traveling from bs->i)
				for(int i = 0; i < input->getN(); i++) {
					lhs += D_bi.at(i)*E_lik_b.at(l).at(i).at(k)*(1.0/input->getV_l(l))*input->getRho_m(l);
				}

				// Add in time to service node i
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						lhs += Ts_i.at(i)*E_lijk.at(l).at(j).at(i).at(k)*input->getRho_h(l);
					}
				}

				model.addQConstr(lhs <= input->getB_l(l), "pT_"+itos(l)+itos(k)+"_leq_b");
			}
		}



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
				printf(" %d : %f (%f, %f, %f)\n", i, U_i.at(i).get(GRB_DoubleAttr_X), X_i.at(i).get(GRB_DoubleAttr_X), Y_i.at(i).get(GRB_DoubleAttr_X), Z_i.at(i).get(GRB_DoubleAttr_X));
			}

			// Tour variables...
			printf("Tour:\n");
			for(int l = 0; l < input->getM(); l++) {
				for(int k = 0; k < input->getN(); k++) {
					if(W_lk.at(l).at(k).get(GRB_DoubleAttr_X) > 0.5) {
						printf(" %d runs sub-tour %d\n", l, k);
					}
				}
			}
			for(int l = 0; l < input->getM(); l++) {
				for(int i = 0; i < input->getN(); i++) {
					for(int k = 0; k < input->getN(); k++) {
						if(Eb_lik.at(l).at(i).at(k).get(GRB_DoubleAttr_X) > 0.5) {
							printf(" %d goes BS->%d on %d\n", l, i, k);
						}
						if(E_lik_b.at(l).at(i).at(k).get(GRB_DoubleAttr_X) > 0.5) {
							printf(" %d goes %d->BS on %d\n", l, i, k);
						}
					}
				}
			}
			for(int l = 0; l < input->getM(); l++) {
				for(int i = 0; i < input->getN(); i++) {
					for(int j = 0; j < input->getN(); j++) {
						for(int k = 0; k < input->getN(); k++) {
							if(E_lijk.at(l).at(i).at(j).at(k).get(GRB_DoubleAttr_X) > 0.5) {
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
