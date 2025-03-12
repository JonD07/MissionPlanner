#include "Solver_Tabs.h"


Solver_Tabs::Solver_Tabs() {
	if(SANITY_PRINT)
		printf("Hello from Standard Solver!\n");
}


void Solver_Tabs::Solve(Input* input, Solution* I_crnt) {
//	// Cast pointer to be input
//	Online_Input* onlineInput = (Online_Input*)input;
//
//	int n_k = onlineInput->getNk();
//	const std::vector<int>* sub_tour = onlineInput->getNextList();
//	int firstNode_i = onlineInput->getFirst_i();
//
//	for(int j = 0; j < n_k; j++) {
//		// Node ID
//		int i = sub_tour->at(j);
//	}
//
//	// Save the solution into the cords array
//	HoveringLocation hl(onlineInput->getX_i(), Y_f.get(GRB_DoubleAttr_X), Z_f.get(GRB_DoubleAttr_X), firstNode_i);
//	I_crnt->AddHL(hl);
//	for(int j = 0; j < n_k; j++) {
//		HoveringLocation hl(X_j.at(j).get(GRB_DoubleAttr_X), Y_j.at(j).get(GRB_DoubleAttr_X), Z_j.at(j).get(GRB_DoubleAttr_X), sub_tour->at(j));
//		I_crnt->AddHL(hl);
//	}


	return;
}



