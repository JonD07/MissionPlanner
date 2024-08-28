/*
 * Node.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Node class, used to build graphs.
 */

#pragma once

#include <math.h>

#include "defines.h"

class Node {
public:
	// TODO: Do we need an empty constructor?
	Node();
	Node(int id, double x, double y, double r);
	Node(const Node &n);
	~Node();

	// Returns the distance (in meters) from this vertex to v
	double GetDistanceTo(Node* v);
	// Get this node's ID
	int getID() { return nID; }
	// Get this node's x coordinate
	double getX() { return fX; }
	// Get this node's y coordinate
	double getY() { return fY; }
	// Get this node's WiFi range
	double getR() { return fR; }
    // Predicted time required to collect data from this node from location x,y
    double collectionTime(double x, double y);

	// Overloaded assignment operator
	Node& operator=(const Node& other);

private:
	int nID;
	double fX, fY, fR;
};
