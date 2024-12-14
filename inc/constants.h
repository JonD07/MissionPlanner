/*
 * constants.h
 *
 * Created by:	Matthew Desaulniers
 * On: 			December 14, 2024
 *
 * Description: Global project constants.
 */

// To easily adjust code for new constraint types
enum Constraint_tx_type{
	single_approximation = 0,  //Single line approximation from lookup table
	pwl,                       //Piecewise linear constraints
	lazy                       //Lazy constraint (TODO rename this to something more accurate.)
};