#include "DroneGenerator.h"

DroneGenerator::DroneGenerator(std::string data_path) {
	/*
	  Expected data file structure:
		1								- Drone ID number (expecting integer)
		v_max_1							- Max speed of drone 1 (m/s)
		bat_1							- Usable battery of drone 1 (J)
		t_bat_1							- Time to swap battery on drone 1 (s)
		c1_1 c2_1 c3_1 c4_1				- P(v) = c1v^{3} + c2v^{2} + c3v + c4
		...
		m
		v_max_m
		bat_m
		t_bat_m
		c1_m c2_m c3_m c4_m

	  Example:
		## Drone energy model from "A Power Consumption Model for Multi-rotor Small Unmanned Aircraft Systems"
		0
		20.0
		111888
		120
		0.0296 0 0 160.017711722
		## Drone energy model from "Looking before Crossing..."
		1
		18.0
		437760
		120
		0.07 0.0391 -13.196 390.95

	 * NOTE: We ignore lines that start with '#'
	 */

	if(DEBUG_DRONE_GEN)
		printf("Reading Drone Data\n");

	// Read status
	bool read_success = true;
	// File reader, for handling text file input
	FileReader fileReader(data_path);

	int drone_count = 0;

	std::string id_line;
	while(fileReader.GetNextLine(&id_line) && read_success) {
		int id = 0;
		std::stringstream idStream(id_line);
		idStream >> id;

		double v_max_m, bat_m, t_bat_m, c1_m, c2_m, c3_m, c4_m;
		std::string drone_data_line;
		// Get max speed
		if(fileReader.GetNextLine(&drone_data_line)) {
			std::stringstream dataStream(drone_data_line);
			dataStream >> v_max_m;
		}
		else {
			// Line reading failed
			read_success = false;
		}
		// Get max usable battery
		if(read_success && fileReader.GetNextLine(&drone_data_line)) {
			std::stringstream dataStream(drone_data_line);
			dataStream >> bat_m;
		}
		else {
			// Line reading failed
			read_success = false;
		}
		// Get battery swap time
		if(read_success && fileReader.GetNextLine(&drone_data_line)) {
			std::stringstream dataStream(drone_data_line);
			dataStream >> t_bat_m;
		}
		else {
			// Line reading failed
			read_success = false;
		}
		// Get energy usage function
		if(read_success && fileReader.GetNextLine(&drone_data_line)) {
			std::stringstream dataStream(drone_data_line);
			dataStream >> c1_m;
			dataStream >> c2_m;
			dataStream >> c3_m;
			dataStream >> c4_m;
		}
		else {
			// Line reading failed
			read_success = false;
		}

		// If we made it this far... we have enough info for a drone type
		DroneType type(id, v_max_m, bat_m, t_bat_m, c1_m, c2_m, c3_m, c4_m);
		droneTypes.insert(std::pair<int, DroneType>(id, type));

		// Keeping count... for my sanity
		drone_count++;

		if(DEBUG_DRONE_GEN)
				printf(" %d: %f %f %f %f %f %f %f\n", id, v_max_m, bat_m, t_bat_m, c1_m, c2_m, c3_m, c4_m);
	}

	if(DEBUG_DRONE_GEN)
		printf("Found %d drone types\n", drone_count);

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[DroneGenerator::DroneGenerator] : Drone data file format off\n");
		exit(1);
	}
	else {
		if(SANITY_PRINT)
			printf("Successfully read drone data\n");
	}
}

DroneGenerator::~DroneGenerator() {}

// Takes in a node string and returns a node. The string is expected to be in this format:
//  x_1 y_1 z_1 zs_1 q_1 t_1
Drone* DroneGenerator::GenerateDrone(int type_ID, double usable_speed, double usable_bat) {
	if(droneTypes.size() == 0) {
		// Asked for a drone but we do not know of any drone types...
		fprintf(stderr, "[DroneGenerator::GenerateDrone] : No drone data!\n");
		exit(1);
	}

	// Were we given speed and battery data?
	if(usable_speed > 0) {
		// Use said data...
		if(droneTypes.count(type_ID) > 0) {
			return new Drone(usable_speed, usable_bat, droneTypes.at(type_ID));
		}
		else {
			// Bad type, just assume it is type 0
			return new Drone(usable_speed, usable_bat, droneTypes.at(0));
		}
	}
	else {
		// Not specified, just use max values
		if(droneTypes.count(type_ID) > 0) {
			return new Drone(droneTypes.at(type_ID).max_speed, droneTypes.at(type_ID).usable_jules, droneTypes.at(type_ID));
		}
		else {
			// Bad type, just assume it is type 0
			return new Drone(droneTypes.at(0).max_speed, droneTypes.at(0).usable_jules, droneTypes.at(0));
		}
	}
}
