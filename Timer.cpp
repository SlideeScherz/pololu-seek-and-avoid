// 
// 
// 

#include "Timer.h"

Timer::Timer(unsigned long _period)
{
  period = _period;
}

bool Timer::ready()
{
  t1 = millis();
  return t1 > t2 + period;
}

void Timer::reset()
{ t2 = t1; }
