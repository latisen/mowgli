#include "sensing.h"
#include "adcman.h"
#include "definition.h"

bool SENSING::obstacleFront(){
  return false;
}
int value;
int SENSING::getBatteryLevel(){
  //int value = ADCMan.read(SOC_PIN);
  //unsigned long newReading = ADCMan.read(pinBatteryVoltage);
  //newReading = newReading * 9;
  //newReading = newReading * 10;
  //Serial.println(value);
  //newReading /= 10000;
  //return newReading;
  //return word(value);

}

bool SENSING::needCharge(){
  if(getBatteryLevel() < batteryEmptyLevel){
    return true;
  }else{
    return false;
  }
}

bool SENSING::isInCharger(){
  if(digitalRead(DOCK_PIN) == true){
    return true;
  }else{
    return false;
  }
}

bool SENSING::isFullyCharged(){
  if(getBatteryLevel() > 13.00){
    return true;
  }else{
    return false;
  }
}
