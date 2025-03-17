#include "Drone.h"

Drone::Drone(double set_speed, double bat_share, const DroneType &type) : droneType(type) {
	setSpeed = set_speed;
	batShare = bat_share;
}

Drone::Drone(const Drone &d) : droneType(d.droneType) {
	setSpeed = d.setSpeed;
	batShare = d.batShare;
}

Drone::~Drone() {}

Drone& Drone::operator=(const Drone& d) {
	// Guard self assignment
	if (this == &d)
		return *this;

	setSpeed = d.setSpeed;
	batShare = d.batShare;
	droneType = DroneType(d.droneType);

	return *this;
}

double Drone::GetRhoM() {
	// y = c1x^{3} + c2x^{2} + c3x + c4
	return droneType.c1*pow(setSpeed, 3) + droneType.c2*pow(setSpeed, 2) + droneType.c3*setSpeed + droneType.c4;
}

double Drone::GetRhoH() {
	return droneType.c4;
}
