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

#define DEBUG_DRONE_GEN	DEBUG || 0


class DroneGenerator {
public:
	DroneGenerator(std::string data_path);
	virtual ~DroneGenerator();

	// Takes in a Drone string and returns a drone. The string is expected to be in this format:
	//  TODO (I think this needs to change): x_1 y_1 z_1 zs_1 q_1 t_1
	
	Drone* GenerateDrone(int type, double usable_speed = -1, double bat_share = -1);
private:
	// List of node parameters
	std::map<int, DroneType> droneTypes;
};
