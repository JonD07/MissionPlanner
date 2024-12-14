#pragma once

#include "gurobi_c++.h"


class callback_class: public GRBCallback{
public:
callback_class();
~callback_class(){};
callback_class(std::vector<GRBVar>* R_new, std::vector<GRBVar>* D_new);

std::vector<GRBVar> R;
std::vector<GRBVar> D;

protected:
void callback();
};