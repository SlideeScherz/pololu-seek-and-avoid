// Timer.h

#ifndef _TIMER_h
#define _TIMER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct Timer {
public:
	
	unsigned long t1;
	unsigned long t2;
	unsigned long period;

	Timer(unsigned long _period);

	bool ready();

	void reset();
};

#endif

