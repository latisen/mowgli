#include "movement.h"
#include "perimeter.h"
#include "Definition.h"
#include "adcman.h"
#include "robot.h"

static int l_speed = 0;
static int l_oldspeed = 0;
static int r_speed = 0;
static int r_oldspeed = 0;
unsigned long previousLeftMotorMillis = 0;
unsigned long previousRightMotorMillis = 0;
unsigned long previousTestMillis = 0;
int motorUpdateInterval = 50;
int testInterval = 1000;
int ledstate;
int pot;

MOVEMENT::MOVEMENT(Perimeter* perimeter){

}

void MOVEMENT::moveForward(int leftSpeed, int rightSpeed) {
  runLeftMotor(leftSpeed, true);
  runRightMotor(rightSpeed, true);
}

void MOVEMENT::runRightMotor(int setSpeed, bool setDirection){
  currentRobotMillis = millis();
  if(setSpeed == r_speed){
  }else if(setSpeed > r_speed){
    //rampa upp / ner motorn
    if (currentRobotMillis - previousRightMotorMillis >= motorUpdateInterval) {
      r_oldspeed = r_speed;
      if(r_speed >= setSpeed){
        r_speed = setSpeed;
      }else{
        r_speed = r_oldspeed + 2;
      }
        analogWrite(WHEEL_MOTOR_A_PWM_PIN, 2.55*abs(r_speed));
        digitalWrite(WHEEL_MOTOR_A_DIRECTION_PIN, setDirection);
        previousRightMotorMillis = currentRobotMillis;
    }
  }else if(setSpeed < r_speed){
    if (currentRobotMillis - previousRightMotorMillis >= motorUpdateInterval) {
      r_oldspeed = r_speed;
      if(r_speed >= setSpeed){
        r_speed = setSpeed;
      }else{
        r_speed = r_oldspeed - 2;
      }
        analogWrite(WHEEL_MOTOR_A_PWM_PIN, 2.55*abs(r_speed));
        digitalWrite(WHEEL_MOTOR_A_DIRECTION_PIN, setDirection);
        previousRightMotorMillis = currentRobotMillis;
    }
  }
}

void MOVEMENT::moveBackward(){
  //Instructions to move backward
  runLeftMotor(75, false);
  runRightMotor(75, false);
}

void MOVEMENT::turnLeft(int turntime){
  //Instructions to turn left
  turntime = random(500, 2000);
  while(turntime > 0){
    runLeftMotor(50, false);
    runRightMotor(50, true);
    delay(1);
    turntime--;
  }
}

void MOVEMENT::turnRight(int turntime){
  //Instructions to turn right
  turntime = random(500, 2000);
  while(turntime > 0){
    runLeftMotor(50, true);
    runRightMotor(50, false);
    delay(1);
    turntime--;
  }
}

int MOVEMENT::GoBackwardUntilInside(int inside){
  // Check if tiltAngle is greater then slopeangle, if not return directly.
  while(Robot.perimeter.isInside(0) == false){
    moveBackward();
    ADCMan.run();
    Serial.print("Magn: ");
    Serial.print(Robot.perimeter.getMagnitude(0));
  }
  return 0;
}

void MOVEMENT::stopMoving(){
  runLeftMotor(0, false);
  runRightMotor(0, false);
}

void MOVEMENT::runLeftMotor(int setSpeed, bool setDirection){
  currentRobotMillis = millis();
  if(setSpeed == l_speed){
  }else if(setSpeed > l_speed){
    //rampa upp / ner motorn
    if (currentRobotMillis - previousLeftMotorMillis >= motorUpdateInterval) {
        l_oldspeed = l_speed;
        if(l_speed >= setSpeed){
          l_speed = setSpeed;
        }else{
          l_speed = l_oldspeed + 2;
        }
        analogWrite(WHEEL_MOTOR_B_PWM_PIN, 2.55*abs(l_speed));
        digitalWrite(WHEEL_MOTOR_B_DIRECTION_PIN, setDirection);
        previousLeftMotorMillis = currentRobotMillis;
    }
  }else if(setSpeed < l_speed){
    if (currentRobotMillis - previousLeftMotorMillis >= motorUpdateInterval) {
        l_oldspeed = l_speed;
        if(l_speed >= setSpeed){
          l_speed = setSpeed;
        }else{
          l_speed = l_oldspeed - 2;
        }
        analogWrite(WHEEL_MOTOR_B_PWM_PIN, 2.55*abs(l_speed));
        digitalWrite(WHEEL_MOTOR_B_DIRECTION_PIN, setDirection);
        previousLeftMotorMillis = currentRobotMillis;
    }
  }
}

void MOVEMENT::blinkLED(){
  currentRobotMillis = millis();
}
