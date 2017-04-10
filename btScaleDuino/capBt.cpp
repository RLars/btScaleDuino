#include "CapBt.hpp"

CapBt::CapBt(CapacitiveSensor * buttonObj, const char *buttonName, void(*cb)(void), unsigned long sensorRes, unsigned long debounceTime) : 
  buttonObj(buttonObj), buttonName(buttonName), cb(cb), sensorRes(sensorRes), debounceTime(debounceTime)
{
  csSum = 0;
  debounceMillis = 0;
  buttonObj->set_CS_AutocaL_Millis(0xFFFFFFFF); // turn off autocalibrate on channel 1
}

void CapBt::process()
{
  long cs = buttonObj->capacitiveSensor(sensorRes); //a: Sensor resolution is set to sensorRes
  if (cs > 6) { //b: Arbitrary number
    csSum += cs;
   
    if (csSum >= 30) //c: This value is the threshold, a High value means it takes longer to trigger
    {
      #if CAP_BT_LOGGING
      Serial.print("Trigger: ");
      Serial.print(csSum);
      Serial.print(" on Button ");
      Serial.println(buttonName);
      #endif

      /* Trigger callback, if debouncing time has passed */
      if ((millis() - debounceMillis) > debounceTime)
      {
        cb();
        debounceMillis = millis();
      }
      
      if (csSum > 0) { csSum = 0; } //Reset
      buttonObj->reset_CS_AutoCal(); //Stops readings
    }
  } else {
    csSum = 0; //Timeout caused by bad readings
    //Serial.print("X");
    //Serial.println(csSum);
  }
}

