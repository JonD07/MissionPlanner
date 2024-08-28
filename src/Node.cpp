#include "Node.h"

Node::Node() : nID(-1), fX(0), fY(0), fR(0) {}

Node::Node(int id, double x, double y, double r) : nID(id), fX(x), fY(y), fR(r) {}

Node::Node(const Node &n) {
	nID = n.nID;
	fX = n.fX;
	fY = n.fY;
	fR = n.fR;
}

Node::~Node() {}

Node& Node::operator=(const Node& other) {
	// Guard self assignment
	if (this == &other)
		return *this;

	this->nID = other.nID;
	this->fX = other.fX;
	this->fY = other.fY;

	return *this;
}

double Node::GetDistanceTo(Node* n) {
	return sqrt(pow((fX - n->fX), 2) + pow((fY - n->fY), 2));
}

//// Budget cost to collect data from this node: t * s_m/s_h,
//// which is [ t * (total time moving at V_MAX)/(total hovering time) ]
//double Node::sensorCost(Location &i) {
//	return sensorTime(i)*((Q) / (HOVER_TIME));
//}
//
//// Actual hovering time, in seconds
//double Node::sensorTime(Location &l) {return 5;}

// Predicted time required to collect data from this node from location x,y
// TODO: Fix this!!
double Node::collectionTime(double x, double y) {
	return 5.0;
}
