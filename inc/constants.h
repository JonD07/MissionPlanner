/*
 * constants.h
 *
 * Created by:	Matthew Desaulniers
 * On: 			December 14, 2024
 *
 * Description: Global project constants.
 */

#pragma once

// To easily adjust code for new constraint types
enum Constraint_tx_type{
	SINGLE_APPROXIMATION = 0,  //Single line approximation from lookup table
	PWL,                       //Piecewise linear constraints
	LAZY                       //Lazy constraint (TODO rename this to something more accurate.)
};