// Clock.h

#ifndef _CLOCK_h
#define _CLOCK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class Clock
{
public:
	// time task began
	unsigned long t1;

	// time task ended
	unsigned long t2;

	unsigned long interval;
	unsigned long taskPeriod;
	
	// status flags
	bool _ready;
	bool _running;
	bool _blocked;
	
	Clock(unsigned long period);
	
	bool ready();

	void start();

	bool completed();

	void reset();
};
#endif