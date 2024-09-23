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
void Node::getTXParams(double* a, double* b, double* mrate) {
	*a = oNodeParameters.A;
	*b = oNodeParameters.B;
	*mrate = oNodeParameters.maxRate;
}

// Get the "agnostic" max TX range
double Node::getR() {
	return oNodeParameters.R;
}

// Predicted time required to collect data from this node from location x,y,z
double Node::collectionTime(double x, double y, double z) {
	double dist = sqrt(pow(fX-x, 2) + pow(fY-y, 2) + pow(fZ-z, 2));
	double rate = std::max(oNodeParameters.maxRate, oNodeParameters.A/(dist*dist) + oNodeParameters.B);
	return fQ/rate;
}
