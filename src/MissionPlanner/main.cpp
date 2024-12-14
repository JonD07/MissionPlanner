#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>

#include "defines.h"
#include "Offline_Input.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Baseline.h"
#include "Solver_Greedy.h"
#include "Solver_Opt.h"
#include "Solver_VRP.h"
#include "COptimizer.h"
#include "TighteningHeuristic.h"
#include "DummyHeuristic.h"




#define DEBUG_MAIN	DEBUG || 0

#define REC_COMP_Z		0
#define ESTIMATE_Z		1
#define PRINT_RESULTS	0
#define PRINT_SUBTOURS	0
#define DATA_LOG_FORMAT	"alg_%d.dat"
#define DATA_LOG_DEFLT_PATH	""
#define NODE_DATA_PATH	"../data/node_data.dat"
#define DRONE_DATA_PATH	"../data/drone_data.dat"


enum {
	e_Algo_OPTIMAL = 0,
	e_Algo_GREEDY = 1,
	e_Algo_APX_GREEDY = 2,
	e_Algo_VRP = 3,
	e_Algo_BASELINE = 4,
	e_Algo_BL_NO_IMPRV = 5,
};



int main(int argc, char *argv[]) {
	// Run parameters
	int algorithm = 0;
	bool printPlan = false;
	bool printResults = false;
	std::string outputPath = "";
	int run_number = 0;

	// Verify user input
	if(argc < 2) {
		fprintf(stderr, "Received %d args, expected 2 or more.\nExpected use:\t./mission-planner <file path> [algorithm] [print plan] [print results] [file path] [run number]\n\n", (argc-1));
		exit(1);
	}

	if(argc == 3) {
		algorithm = atoi(argv[2]);
	}
	else if(argc == 4) {
		algorithm = atoi(argv[2]);
		printPlan = atoi(argv[3]);
	}
	else if(argc == 5) {
		algorithm = atoi(argv[2]);
		printPlan = atoi(argv[3]);
		printResults = atoi(argv[4]);
	}
	else if(argc == 6) {
		algorithm = atoi(argv[2]);
		printPlan = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = std::string(argv[5]);
	}
	else if(argc == 7) {
		algorithm = atoi(argv[2]);
		printPlan = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = std::string(argv[5]);
		run_number = atoi(argv[6]);
	}

	// Just to keep things consistent...
	srand(run_number);

	Solver* solver = NULL;
	Offline_Input input(argv[1]);
	Solution solution(&input);
	COptimizer cOptimizer(tx_curve_lazy);
	TighteningHeuristic tHeuristic;
	DummyHeuristic dHeuristic;

	switch(algorithm) {
	// Optimal solver
	case e_Algo_OPTIMAL: {
		solver = new Solver_Opt();
	}
	break;

	// Greedy solver (more for mucking about with code base than actually useful) DummyHeuristic
	case e_Algo_GREEDY: {
		solver = new Solver_Greedy();
	}
	break;

	// Greedy solver (more for mucking about with code base than actually useful)
	case e_Algo_APX_GREEDY: {
		solver = new Solver_Greedy(true);
	}
	break;

	// Clustering followed by VRP algorithm (HL optimizing)
	case e_Algo_VRP: {
		solver = new Solver_VRP(&cOptimizer);
	}
	break;

	// Clustering followed by VRP algorithm (tightening heuristic)
	case e_Algo_BASELINE: {
		solver = new Solver_VRP(&tHeuristic);
	}
	break;

	// Clustering followed by VRP algorithm (tightening heuristic)
	case e_Algo_BL_NO_IMPRV: {
		solver = new Solver_VRP(&dHeuristic);
	}
	break;

	default:
		// No valid algorithm given
		fprintf(stderr, "[ERROR][main] : \n\tInvalid algorithm identifier!\n");
		exit(1);
	}

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();

	// Run the solver
	solver->Solve(&input, &solution);

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();
	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	double duration_s = (double)duration/1000.0;

	// Results
	double result = solution.Benchmark();

	if(SANITY_PRINT) {
		printf("\nFound solution: %f\n", result);
		printf("Computation time = %f s\n", duration_s);
		solution.PrintSolution();
	}

	// Print results to file
	if(printResults) {
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath.c_str());
		sprintf(buff + strlen(buff), DATA_LOG_FORMAT, algorithm);
		if(SANITY_PRINT)
			printf(" Printing results to: %s\n", buff);
		pOutputFile = fopen(buff, "a");
		// File format: n m runnum Objective comp-time data_size
		fprintf(pOutputFile, "%d %d %d ", input.getN(), input.getM(), run_number);
		fprintf(pOutputFile, "%.10f %f ", result, duration_s);
		fprintf(pOutputFile, "%f ", input.getQ_i(0));
		// Print if this is a valid solution
		fprintf(pOutputFile, "%d ", solution.ValidSolution(true));
		

		if(PRINT_SUBTOURS) {
			// Grab sub-tour times
			std::vector<std::pair<std::string,double>> sub_tours;
			solution.GetSubTourTimes(&sub_tours);

			// Print these too
			for(std::pair<std::string,double> p : sub_tours) {
				fprintf(pOutputFile, "%s %f ", p.first.c_str(), p.second);
			}

			// Print out what the tours are
			std::vector<std::string> sub_tour_string;
			solution.GetSubTours(&sub_tour_string);
			for(std::string s : sub_tour_string) {
				fprintf(pOutputFile, "%s ", s.c_str());
			}

		}
		fprintf(pOutputFile, "\n");

		fclose(pOutputFile);
	}

	// Print flight plan?
	if(printPlan) {
		// Yes, print flight plan!
		solution.PrintPlan();
	}

	delete solver;

	if(SANITY_PRINT)
		printf("Done\n");
}
