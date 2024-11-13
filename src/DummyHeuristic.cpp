#include "DummyHeuristic.h"


DummyHeuristic::DummyHeuristic() {}

// Attempts to improve hovering location positions using the tightening heuristic. Returns false if the solution would violate the drone's energy limit.
bool DummyHeuristic::ImproveSubTour(int l, Input* input, std::vector<Point>* og_sub_tour, bool dummy_flag) {
	if(DEBUG_DH)
		printf("Dummy heuristic does nothing..\n");

	return true;
}
