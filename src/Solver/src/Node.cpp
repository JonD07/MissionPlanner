#include "Node.h"

Node::Node() : nID(-1), fX(0), fY(0), fZ(0), fZSafe(0), fQ(0), nNodeType(-1), sIP("localhost") {}

Node::Node(int id, double x, double y, double z, double zs, double q, int type, NodeParameters& nodeParameters, std::string ip) :
		nID(id), fX(x), fY(y), fZ(z), fZSafe(zs), fQ(q), nNodeType(type), oNodeParameters(nodeParameters), sIP(ip) {}


Node::Node(const Node &n) {
	nID = n.nID;
	fX = n.fX;
	fY = n.fY;
	fZ = n.fZ;
	fZSafe = n.fZSafe;
	fQ = n.fQ;
	nNodeType = n.nNodeType;
	oNodeParameters = n.oNodeParameters;
	sIP = n.sIP;
}

Node::~Node() {}

Node& Node::operator=(const Node& other) {
	// Guard self assignment
	if (this == &other)
		return *this;

	nID = other.nID;
	fX = other.fX;
	fY = other.fY;
	fZ = other.fZ;
	fZSafe = other.fZSafe;
	fQ = other.fQ;
	nNodeType = other.nNodeType;
	oNodeParameters = other.oNodeParameters;
	sIP = other.sIP;

	return *this;
}

double Node::GetDistanceTo(Node* n) {
	return sqrt(pow((fX - n->fX), 2) + pow((fY - n->fY), 2));
}

// Get data TX parameters
void Node::getTXParams(double* a, double* b, double* max_rate, double* C, double* min_rate) {
	*a = oNodeParameters.A;
	*b = oNodeParameters.B;
	*max_rate = oNodeParameters.maxRate;
	*C = oNodeParameters.C;
	*min_rate = oNodeParameters.minRate;
}

// Get the "agnostic" max TX range
double Node::getR() {
	return oNodeParameters.R;
}

// Given drone velocity, lookup the optimal range to communicate with sensor
double Node::getOptimalRange(double velocity) {
	// We will round everything down
	//use sizeof to clean up these hardcoded constants. Explanation of these numbers is in input.h
	int velocity_index = int(velocity) - 2;
	int q_size_index = int( log2(fQ/0.001)*8 );
	double distance = pi_q_vs_distance_lookup[nNodeType][velocity_index][q_size_index];
	return distance;
}

// Predicted time required to collect data from this node from location x,y,z
double Node::collectionTime(double x, double y, double z) {
	double dist = sqrt(pow(fX-x, 2) + pow(fY-y, 2) + pow(fZ-z, 2));
	double rate = std::min(oNodeParameters.maxRate, oNodeParameters.A/(dist*dist+oNodeParameters.C) + oNodeParameters.B);
	return fQ/rate;
}
