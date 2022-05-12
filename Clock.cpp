// 
// 
// 

#include "Clock.h"

Clock::Clock(unsigned long period)
{
  interval = period;
  taskPeriod = 0ul;
  _ready = false;
  _running = false;
  _blocked = false;
}

bool Clock::ready()
{
  _ready = millis() > t2 + interval;

  return _ready;
}

void Clock::start()
{
  _ready = false;
  _running = true;
  t1 = millis();
}

bool Clock::completed()
{
  if (_running)
  {
    return millis() > t1 + taskPeriod;
  }
}

void Clock::reset() 
{ 
  _ready = true;
  _running = false;
  t2 = millis();
}
