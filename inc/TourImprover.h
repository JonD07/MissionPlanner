/*
 * TourImprover.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 8, 2024
 *
 * Description: Base class for sub-tour improvement algorithms.
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <list>
#include <time.h>
#include <utility>

#include "Utilities.h"
#include "Solver.h"

#define DEBUG_TI		DEBUG || 0
#define is_zero(x)	((x<0.001)&&(x>-0.001))


// A point in 3D space
struct Point {
	double x, y, z;
	int node_id;

	Point(int id, double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
		node_id = id;
	}
	Point(const Point& other) {
		x = other.x;
		y = other.y;
		z = other.z;
		node_id = other.node_id;
	}
	Point& operator=(const Point& other) {
		x = other.x;
		y = other.y;
		z = other.z;
		node_id = other.node_id;

	    return *this;
	}



	// Subtract two points to get a vector
	Point operator-(const Point& other) const {
		return {node_id, x - other.x, y - other.y, z - other.z};
	}

	// Add a scaled vector to a point
	Point operator+(const Point& vec) const {
		return {node_id, x + vec.x, y + vec.y, z + vec.z};
	}

	// Scale a vector by a scalar
	Point operator*(double scalar) const {
		return {node_id, x * scalar, y * scalar, z * scalar};
	}

	// Hadamard division for vectors
	Point operator/(const Point& vec) const {
		double x_prime, y_prime, z_prime;

		// Element-wise division
		if(is_zero(vec.x)) {
			x_prime = 0.0;
		}
		else {
			x_prime = x / vec.x;
		}
		if(is_zero(vec.y)) {
			y_prime = 0.0;
		}
		else {
			y_prime = y / vec.y;
		}
		if(is_zero(vec.z)) {
			z_prime = 0.0;
		}
		else {
			z_prime = z / vec.z;
		}

		return {node_id, x_prime, y_prime, z_prime};
	}

	// Dot product between two vectors
	double dot(const Point& other) const {
		return x * other.x + y * other.y + z * other.z;
	}

	// L2 norm (distance)
	double l2(const Point& other) const {
		return distAtoB(x, y, z, other.x, other.y, other.z);
	}

	// Calculate average among all non-zero elements in this point
	double calc_z() const {
		double z = 0;
		int count = 0;

		// Add up all non-zero elements
		if(!is_zero(x)) {
			z += x;
			count++;
		}
		if(!is_zero(y)) {
			z += y;
			count++;
		}
		if(!is_zero(z)) {
			z += z;
			count++;
		}

		if(count > 0) {
			return z / count;
		}
		else {
			return 0.0;
		}
	}

	// Calculate the magnitude of the vector
	double magnitude() const {
		return std::sqrt(x * x + y * y + z * z);
	}

	// Calculate the normalized vector
	Point normalize() const {
		double mag = magnitude();
		return {0, x / mag, y / mag, z / mag};
	}

	// Function to find a point at distance 'r' from this point towards another point p2
	Point pointAtDistance(const Point& p2, double r) const {
		// Vector from this point to p2
		Point direction = p2 - *this;

		// Normalize the direction vector
		Point direction_unit = direction.normalize();

		// Return the point p' = p1 + r * direction_unit
		return *this + direction_unit * r;
	}

	// Print a point
	void print() const {
		if(DEBUG_TI)
			std::cout << node_id << ":(" << x << ", " << y << ", " << z << ")" << std::endl;
	}
};


class TourImprover {
public:
	TourImprover();
	virtual ~TourImprover();

	// Used to improve a sub-tour  TODO: Remove this flag... we don't really use it
	virtual bool ImproveSubTour(int l, Input* input, std::vector<Point>* sub_tour, bool additional_flag = false) = 0;

protected:
private:
};
