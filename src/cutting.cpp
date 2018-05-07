#include "cutting.h"
#include "definition.h"

void CUTTING::startCutting(int setSpeed){
  analogWrite(CUTTER_PWM_PIN, setSpeed);
}

void CUTTING::stopCutting(){
  analogWrite(CUTTER_PWM_PIN, 0);
}
