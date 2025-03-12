/*
 * Online_Input.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 22, 2024
 *
 * Description: Input for online solver
 *
 */

#pragma once

#include "Input.h"


#define DEBUG_ONLINEINPUT	DEBUG || 0


class Online_Input : public Input {
public:
	Online_Input(std::string scenario_input_path);
	virtual ~Online_Input();

	/// Getters
	// Number of follow-on nodes to visit
	int getNk();
	// Get the operational speed of the drone
	double getV();
	// Get the time to swap batteries of drone l
	double getTb();
	// Get rho for moving for drone l
	double getRho_m();
	// Get rho for moving for drone l
	double getRho_h();
	// Get beta for drone l (planning energy budget, in Jule)
	double getB();
	// Get the x-coordinate of the drones current position
	double getX_s() { return currnt_x; }
	// Get the y-coordinate of the drones current position
	double getY_s() { return currnt_y; }
	// Get the z-coordinate of the drones current position
	double getZ_s() { return currnt_z; }
	// Get the x-coordinate of the first node to visit
	double getX_f();
	// Get the y-coordinate of the first node to visit
	double getY_f();
	// Get the z-coordinate of the first node to visit
	double getZ_f();
	// Get the index of the first node
	int getFirst_i() { return first_node; }
	// Get the list of follow-on nodes to visit
	const std::vector<int>* getNextList() { return &next_list; }

protected:

private:
	int drone_type = 0;
	double currnt_x = 0.0, currnt_y = 0.0, currnt_z = 0.0;
	int first_node = 0;
	std::vector<int> next_list;
};
