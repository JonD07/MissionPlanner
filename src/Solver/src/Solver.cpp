#include "Solver.h"

using namespace std::complex_literals;

Solver::Solver() {
}

Solver::~Solver() {}

bool Solver::wholeNumber(double f) {
	double diff = f-floor(f);
	if(diff >= 0) {
		return diff < 0.0005;
	}
	else {
		return diff > -0.0005;
	}
}
