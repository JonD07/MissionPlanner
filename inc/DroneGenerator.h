/*
 * DroneGenerator.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 10, 2024
 *
 * Description: Drone generator class, used to create drone objects. The
 * drone generator reads in a drone types file and assigns parameters to
 * the generated drone based on these parameters.
 *
 */

#pragma once

#include <map>
#include <sstream>
#include <fstream>

#include "defines.h"
#include "Drone.h"
#include "FileReader.h"

#define DEBUG_DRONE_GEN	DEBUG || 1


class DroneGenerator {
public:
	DroneGenerator(std::string data_path);
	virtual ~DroneGenerator();

	// Takes in a node string and returns a node. The string is expected to be in this format:
	//  x_1 y_1 z_1 zs_1 q_1 t_1
	Drone* GenerateDrone(int type, double usable_speed, double usable_bat);
private:
	// List of node parameters
	std::map<int, DroneType> droneTypes;
};
