#ifndef __CAP_BT_HPP
#define __CAP_BT_HPP

#include <CapacitiveSensor.h>

#define CAP_BT_LOGGING 0

class CapBt
{
public:
  CapBt(CapacitiveSensor * buttonObj, const char *buttonName, void(*cb)(void), unsigned long sensorRes, unsigned long debounceTime); // constructor
  void process(); // process function  
  
private:
  CapacitiveSensor * buttonObj;
  const char *buttonName;
  void(*cb)(void);
  unsigned long sensorRes;
  unsigned long debounceTime;
  unsigned long csSum;
  unsigned long debounceMillis;
};

#endif // __CAP_BT_HPP
