// 
// 
// 

#include "Coordinate.h"

Coordinate::Coordinate() {
	x = 0.0;
	y = 0.0;
	theta = 0.0;
}

Coordinate::Coordinate(double _x, double _y, double _theta = NULL) {
	x = _x;
	y = _y;
	theta = _theta;
}

