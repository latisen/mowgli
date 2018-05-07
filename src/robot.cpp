#include "robot.h"

#define MAGIC 52



// RobotState {  STAT_IDLE, STAT_CAL_GYRO, STAT_TRACK, STAT_CREATE_MAP, STAT_MOW, STAT_RC };
const char* robotStateNames[] = { "IDLE", "GYRO", "TRAK", "MAP ", "MOW ", "R/C " };



RobotClass::RobotClass(){
}


// robot start
void RobotClass::begin(){
  Wire.begin();
  ADCMan.init();
  Serial.print("Calibrating...");
  ADCMan.calibrate();
  delay(1000);
  Serial.print("Done calibrating...");
  perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);
  perimeter.useDifferentialPerimeterSignal = true;
  perimeter.speedTest();
  //ADCMan.setCapture(testbatpin, 4, true);
  Serial.begin(115200); 						// Fast communication on the serial port

  //Some variables
  nextTimeMotorPerimeterControl = 0;
  MaxSpeedperiPwm = 45;
  trackingPerimeterTransitionTimeOut = 2800;   // never<500 ms
  trackingErrorTimeOut = 10000;  // 0=disable
  trackingBlockInnerWheelWhilePerimeterStruggling = 1;
  stateStartTime = millis();
  perimeterMag = 1;
  perimeterMagMedian.add(perimeterMag);
  perimeterInside = true;
  perimeterCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;
  perimeterPID.Kp            = 16;       // perimeter PID controller
  perimeterPID.Ki            = 8;
  perimeterPID.Kd            = 0.8;
  loopCounter = 0;
  turningTimer = 0;
  nextTimeMotorSense = 0;

  //ADCMan.setCapture(pinChargeCurrent, 1, true);//Aktivierung des LaddeStrom Pins beim ADC-Managers
  ADCMan.setCapture(CUTTER_CURRENT_PIN, 1, true);
  ADCMan.setCapture(WHEEL_MOTOR_A_CURRENT_PIN, 1, true);
  ADCMan.setCapture(WHEEL_MOTOR_B_CURRENT_PIN, 1, true);
  ADCMan.setCapture(SOC_PIN, 1, false);
  //ADCMan.setCapture(pinChargeVoltage, 1, false);
  //ADCMan.setCapture(pinVoltageMeasurement, 1, false);
  perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);

}

float RobotClass::voltageDividerUges(float R1, float R2, float U2){
	return (U2/R2 * (R1+R2));  // Uges
}

float RobotClass::ADC2voltage(float ADCvalue){
  return (ADCvalue /1023.0 * IOREF);   // ADCman works @ 10 bit
}

bool RobotClass::checkInside(int side){
  ADCMan.run();
  if(side == 0){
    perimeter.getMagnitude(0);
    if(perimeter.isInside(0)){
      return true;
    }else{
      return false;
    }
  }

  if(side == 1){
    perimeter.getMagnitude(1);
    if(perimeter.isInside(1)){
      return true;
    }else{
      return false;
    }
  }
  Serial.println(Robot.perimeter.getMagnitude(0));
  Serial.println(Robot.perimeter.getMagnitude(1));
  Serial.print("Left: ");
  Serial.print(Robot.perimeter.isInside(0));
  Serial.print(" Right: ");
  Serial.println(Robot.perimeter.isInside(1));
}

void RobotClass::rotateMower(int dir){
  if(dir == 0){
    while(!perimeter.isInside(1)){
      goBackward(2000);
    }
    if(perimeter.isInside(1)){
      return true;
    }else{
      return false;
    }
    //rotate left
  }

  if(dir == 1){
    //rotate left
  }
}

void RobotClass::goBackward(int howLong){
  starttime = millis();
  endtime = millis() + howLong;
  while(starttime < endtime){
    setLeftPWM = -255;
    setRightPWM = -255;
    accelerateMotor(setLeftPWM, setRightPWM);
  }
  setLeftPWM = 0;
  setLeftPWM = 0;
  accelerateMotor(setLeftPWM, setRightPWM);
}

int i;
int newSetLeftPWM;
int newSetRightPWM;

void RobotClass::accelerateMotor(int setLeftPWM, int setRightPWM){
      currentTime = millis();
      accel = 300; //Time to accelerate in ms
      if (currentTime - lastAccelTime > accel) {
        lastAccelTime = currentTime;
        if (spd < setLeftPWM) {
          spd++;
          digitalWrite(WHEEL_MOTOR_A_DIRECTION_PIN, (spd > 0));
          digitalWrite(WHEEL_MOTOR_B_DIRECTION_PIN, (spd > 0));
          analogWrite(WHEEL_MOTOR_A_PWM_PIN, abs(spd));
          analogWrite(WHEEL_MOTOR_B_PWM_PIN, abs(spd));
        }
      }
}

void RobotClass::readSensors(){
  if (millis() >= nextTimeMotorSense){
    nextTimeMotorSense = millis() +  50;
    double accel = 0.05;
    motorRightSenseADC = readSensor(SEN_MOTOR_RIGHT);
    motorLeftSenseADC = readSensor(SEN_MOTOR_LEFT);
    motorMowSenseADC = readSensor(SEN_MOTOR_MOW);

    motorRightSenseCurrent = motorRightSenseCurrent * (1.0-accel) + ((double)motorRightSenseADC) * motorSenseRightScale * accel;
    motorLeftSenseCurrent = motorLeftSenseCurrent * (1.0-accel) + ((double)motorLeftSenseADC) * motorSenseLeftScale * accel;
    motorMowSenseCurrent = motorMowSenseCurrent * (1.0-accel) + ((double)motorMowSenseADC) * motorMowSenseScale * accel;

    if (batVoltage > 8){
      motorRightSense = motorRightSenseCurrent * batVoltage /1000;   // conversion to power in Watt
      motorLeftSense  = motorLeftSenseCurrent  * batVoltage /1000;
      motorMowSense   = motorMowSenseCurrent   * batVoltage /1000;
    }else{
      motorRightSense = motorRightSenseCurrent * batFull /1000;   // conversion to power in Watt in absence of battery voltage measurement
      motorLeftSense  = motorLeftSenseCurrent  * batFull /1000;
      motorMowSense   = motorMowSenseCurrent   * batFull /1000;
    }
  }

  if ((millis() - lastMotorMowRpmTime) >= 500){
    motorMowRpmCurr = readSensor(SEN_MOTOR_MOW_RPM);
    if ((motorMowRpmCurr == 0) && (motorMowRpmCounter != 0)){
      // rpm may be updated via interrupt
      motorMowRpmCurr = (int) ((((double)motorMowRpmCounter) / ((double)(millis() - lastMotorMowRpmTime))) * 60000.0);
      motorMowRpmCounter = 0;
    }
    lastMotorMowRpmTime = millis();
    if (!ADCMan.calibrationDataAvail()) {
      //Console.println(F("Error: missing ADC calibration data"));
      //addErrorCounter(ERR_ADC_CALIB);
      //setNextState(STATE_ERROR, 0);
    }
  }

}

int RobotClass::readSensor(char type){
  switch (type) {
// motors------------------------------------------------------------------------------------------------
#if defined (DRIVER_MC33926)
    case SEN_MOTOR_MOW: return ADCMan.read(pinMotorMowSense); break;
    case SEN_MOTOR_RIGHT: checkMotorFault(); return ADCMan.read(pinMotorRightSense); break;
    case SEN_MOTOR_LEFT:  checkMotorFault(); return ADCMan.read(pinMotorLeftSense); break;
    //case SEN_MOTOR_MOW_RPM: break; // not used - rpm is upated via interrupt
#endif
// perimeter----------------------------------------------------------------------------------------------
    case SEN_PERIM_LEFT: return perimeter.getMagnitude(0); break;
    //case SEN_PERIM_RIGHT: return Perimeter.getMagnitude(1); break;

// battery------------------------------------------------------------------------------------------------
    case SEN_BAT_VOLTAGE: ADCMan.read(pinVoltageMeasurement);  return ADCMan.read(pinBatteryVoltage); break;
  }
  return 0;
}
