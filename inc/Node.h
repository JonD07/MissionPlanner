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
#include <algorithm>

#include "defines.h"


struct NodeParameters {
	int type_id;
	double A, B, maxRate, sigma, R;

	NodeParameters() {
		type_id = -1;
		A = 0;
		B = 0;
		maxRate = 0;
		sigma = 0;
		R = 0;
	}
	NodeParameters(int id, double a, double b, double maxR, double sig, double r) {
		type_id = id;
		A = a;
		B = b;
		maxRate = maxR;
		sigma = sig;
		R = r;
	}
	NodeParameters(const NodeParameters& other) {
		type_id = other.type_id;
		A = other.A;
		B = other.B;
		maxRate = other.maxRate;
		sigma = other.sigma;
		R = other.R;
	}
	NodeParameters& operator=(const NodeParameters& other) {
		type_id = other.type_id;
		A = other.A;
		B = other.B;
		maxRate = other.maxRate;
		sigma = other.sigma;
		R = other.R;

	    return *this;
	}
};


class Node {
public:
	Node();
	Node(int id, double x, double y, double z, double zs, double q, int type, NodeParameters& nodeParameters);
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
	// Get this node's z coordinate
	double getZ() { return fZ; }
	// Get this node's safe altitude
	double getZs() { return fZSafe; }
	// Get this node's data quantity to collect
	double getQ() { return fQ; }
	// Get data TX parameters
	void getTXParams(double* a, double* b, double* mrate);
	// Get the "agnostic" max TX range
	double getR();


    // Predicted time required to collect data from this node from location x,y
    double collectionTime(double x, double y, double z);

	// Overloaded assignment operator
	Node& operator=(const Node& other);

private:
	int nID;
	// Node position
	double fX, fY, fZ;
	// Safe hover distance
	double fZSafe;
	// Quantity of data to collect
	double fQ;
	// Node type (used for determining communication parameters)
	double nNodeType;
	// Node parameters object
	NodeParameters oNodeParameters;
};








