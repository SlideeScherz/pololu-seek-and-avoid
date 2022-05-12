// Coordinate.h

#ifndef _COORDINATE_h
#define _COORDINATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct Coordinate
{
public:
	double x;
	double y;
	double theta;

	Coordinate();
	
	Coordinate(double _x, double _y, double _theta = NULL);
};
#endif

