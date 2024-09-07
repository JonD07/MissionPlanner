/*
 * Solver_Opt.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 8, 2024
 *
 * Description:
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <list>
#include <time.h>

#include "Utilities.h"
#include "Solver.h"
#include "gurobi_c++.h"

#define DEBUG_SLVR_OPT		DEBUG || 1

#define BIG_M		500000
#define CONST_RELAXATION(X)		X,X+0.1


class Solver_Opt : public Solver {
public:
	Solver_Opt();

	void Solve(Input* input, Solution* I_final);

protected:
private:
};
