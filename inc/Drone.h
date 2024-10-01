/*
 * Drone.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sept 10, 2024
 *
 * Description: Drone class... is a drone
 */

#pragma once

#include <math.h>
#include <algorithm>

#include "defines.h"


struct DroneType {
	int id;
	double max_speed;
	double usable_jules;
	double bat_swap_time;
	// P(v) = c1v^{3} + c2v^{2} + c3v + c4
	double c1, c2, c3, c4;

	DroneType(int id, double max_speed, double usable_jules, double bat_swap_time, double c1, double c2, double c3, double c4) {
		this->id = id;
		this->max_speed = max_speed;
		this->usable_jules = usable_jules;
		this->bat_swap_time = bat_swap_time;
		this->c1 = c1;
		this->c2 = c2;
		this->c3 = c3;
		this->c4 = c4;
	}
	DroneType(const DroneType& other) {
		id = other.id;
		max_speed = other.max_speed;
		usable_jules = other.usable_jules;
		bat_swap_time = other.bat_swap_time;
		c1 = other.c1;
		c2 = other.c2;
		c3 = other.c3;
		c4 = other.c4;
	}
};


class Drone {
public:
//	Drone();
	Drone(double set_speed, double bat_share, const DroneType &type);
	Drone(const Drone &n);
	~Drone();

	// Overloaded assignment operator
    Drone& operator=(const Drone& other);

    // Getters
    double GetSpeed() { return setSpeed; }
    double GetBatShare() { return batShare; }
    double GetPlannableEnergy() { return batShare*droneType.usable_jules; }
    double GetSwapTime() { return droneType.bat_swap_time; }
    double GetRhoM();
    double GetRhoH();

private:
    DroneType droneType;
    double setSpeed;
    double batShare;
};
