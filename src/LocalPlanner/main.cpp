#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>

#include "defines.h"
#include "Online_Input.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Standard.h"


#define DEBUG_MAIN	DEBUG || 0

#define PRINT_PLAN			true
#define PRINT_PERFORMANCE	false

#define DATA_LOG_FORMAT	"alg_%d.dat"

enum {
	e_Algo_STANDARD = 0,
};


int main(int argc, char *argv[]) {
	// Run parameters
	bool print_plan = PRINT_PLAN;
	int algorithm = e_Algo_STANDARD;
	bool print_performance = PRINT_PERFORMANCE;
	std::string outputPath = "";

	// Verify user input
	if(argc < 2) {
		fprintf(stderr, "Received %d args, expected 2 or more.\nExpected use:\t./local-planner <file path> [print plan] [algorithm] [print performance] [file path]\n\n", (argc-1));
		exit(1);
	}

	if(argc == 3) {
		print_plan = atoi(argv[2]);
	}
	else if(argc == 4) {
		print_plan = atoi(argv[2]);
		algorithm = atoi(argv[3]);
	}
	else if(argc == 5) {
		print_plan = atoi(argv[2]);
		algorithm = atoi(argv[3]);
		print_performance = atoi(argv[4]);
	}
	else if(argc == 6) {
		print_plan = atoi(argv[2]);
		algorithm = atoi(argv[3]);
		print_performance = atoi(argv[4]);
		outputPath = std::string(argv[5]);
	}

	Solver* solver = NULL;
	Online_Input input(argv[1]);
	Solution solution(&input);

	switch(algorithm) {
	// Optimal solver
	case e_Algo_STANDARD:
	default: {
		solver = new Solver_Standard();
	}
	break;
	}

	// Are we tracking performance?
	std::chrono::_V2::system_clock::time_point start;
	if(print_performance) {
		// Capture start time
		start = std::chrono::high_resolution_clock::now();
	}

	// Run the solver
	solver->Solve(&input, &solution);

	// Are we tracking performance?
	if(print_performance) {
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

		// Print out performance data
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath.c_str());
		sprintf(buff + strlen(buff), DATA_LOG_FORMAT, algorithm);
		if(SANITY_PRINT)
			printf(" Printing results to: %s\n", buff);
		pOutputFile = fopen(buff, "a");
		// File format: n m runmun computed_Z estimated_Z comp-time
		fprintf(pOutputFile, "%d %.10f %f\n", input.getM(), result, duration_s);
		fclose(pOutputFile);
	}

	// Print flight plan?
	if(print_plan) {
		// Yes, print flight plan!
		solution.PrintPlan(false);
	}

	delete solver;

	if(SANITY_PRINT)
		printf("Done\n");
}
