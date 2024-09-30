/*
 * Offline_Input.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 22, 2024
 *
 * Description: Input for mission planner
 *
 *
 */

#pragma once

#include "Input.h"

#define DEBUG_OFFLINEINPUT	DEBUG || 0


class Offline_Input : public Input {
public:
	Offline_Input(std::string scenario_input_path);
	virtual ~Offline_Input();

protected:

private:
};
