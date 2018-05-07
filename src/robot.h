#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include "pid.h"
#include "adcman.h"
#include "perimeter.h"
#include "drivers.h"
#include "RunningMedian.h"
#include "Definition.h"

// finate state machine states
enum RobotState {  STAT_IDLE, STAT_CAL_GYRO, STAT_TRACK, STAT_CREATE_MAP, STAT_MOW, STAT_RC, STAT_CHG};
typedef enum RobotState RobotState;

// tracking
enum TrackState {  TRK_RUN, TRK_FIND, TRK_ROTATE };
typedef enum TrackState TrackState;

// mowing
enum MowState { MOW_ROTATE, MOW_REV, MOW_LINE, MOW_ENTER_LINE, MOW_AVOID_OBSTACLE, MOW_AVOID_ESCAPE } ;
typedef enum MowState MowState;

// mow pattern
enum MowPattern { PATTERN_NONE, PATTERN_RANDOM, PATTERN_LANES } ;
typedef enum MowPattern MowPattern;

// obstacle side
enum Side { FRONT, BACK };
typedef enum Side Side;

// sensors
enum {
  SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
  SEN_PERIM_LEFT_EXTRA,  // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT_EXTRA, // 0..MAX_PERIMETER
  SEN_LAWN_FRONT,
  SEN_LAWN_BACK,
  SEN_BAT_VOLTAGE,       // Volt * 100
  SEN_CHG_CURRENT,       // Ampere * 100
  SEN_CHG_VOLTAGE,       // Volt * 100
  SEN_MOTOR_LEFT,        // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_RIGHT,       // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_MOW,         // 0..MAX_MOW_CURRENT
  SEN_BUMPER_LEFT,       // LOW = pressed
  SEN_BUMPER_RIGHT,      // LOW = pressed
  SEN_DROP_LEFT,       // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
  SEN_DROP_RIGHT,      // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
  SEN_SONAR_CENTER,      // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_LEFT,        // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_RIGHT,       // 0..SONAR_TRIGGER_DISTANCE
  SEN_BUTTON,            // LOW = pressed
  SEN_IMU,
  SEN_MOTOR_MOW_RPM,
  SEN_RTC,
  SEN_RAIN,
  SEN_TILT,
};


class RobotClass
{
  public:
    float mowingAngle;
    float rotateAngle;
    float trackAngle;
		float reverseSpeedPerc;
		float trackSpeedPerc;
		float trackRotationSpeedPerc;
		float rotationSpeedPerc;
    float mowingDirection;
		uint16_t sensorTriggerStatus; // bitmap of triggered sensors
	  unsigned long lastStartLineTime;
    unsigned long loopCounter;
	  RobotState state;
	  RobotState lastState;
	  TrackState trackState;
		bool trackClockwise;
	  MowState mowState;
		MowState lastMowState;
	  MowPattern mowPattern;
		Side obstacleSide;
	  int loopsPerSec;
		float loopsPerSecSmooth;
    RobotClass();
    void begin();
    void run();
	  String getStateName();
    unsigned long nextTimeMotorPerimeterControl;
    int MaxSpeedperiPwm;
    Perimeter perimeter;
    int trackingPerimeterTransitionTimeOut;
    int trackingErrorTimeOut;
    char trackingBlockInnerWheelWhilePerimeterStruggling;
    unsigned long stateStartTime;
    int perimeterMag;
    RunningMedian perimeterMagMedian = RunningMedian(300);
    boolean perimeterInside;
    int perimeterCounter;
    unsigned long perimeterLastTransitionTime;
    unsigned long perimeterTriggerTime;
    PID perimeterPID;
    int perimeterMagMaxValue;
    int leftSpeedperi;
    int rightSpeedperi;
    float PeriCoeffAccel;
    unsigned long lastTimeForgetWire;
    bool checkInside(int side);
    void accelerateMotor(int setLeftPWM, int setRightPWM);
    void goBackward(int howLong);
    void rotateMower(int dir);
    int turningTimer;
    long starttime;
    long endtime;
    int setLeftPWM;
    int setRightPWM;
    int newSetLeftPWM;
    int newSetRightPWM;
    long howLong;
    int i;
    long currentTime;
    long lastAccelTime;
    int accel;
    int spd;
    unsigned long nextTimeMotorSense;
    int motorRightSenseADC ;
    int motorLeftSenseADC ;
    int motorMowSenseADC ;
    float motorRightSenseCurrent ;
    float motorSenseRightScale ;
    float motorLeftSenseCurrent ;
    float motorSenseLeftScale ;
    float motorMowSenseCurrent ;
    float motorMowSenseScale ;
    float batVoltage ;
    float motorLeftSense ;      // motor power (range 0..MAX_MOTOR_POWER)
    float motorRightSense;
    float motorMowSense;
    float batFull;
    void readSensors();
    unsigned long lastMotorMowRpmTime;
    int motorMowRpmCurr ;
    int motorMowRpmCounter ;
    float voltageDividerUges(float R1, float R2, float U2);
    float ADC2voltage(float ADCvalue);

  protected:
    int lastPerimeterMag;
	  unsigned long trackLineTimeout;
    unsigned long nextInfoTime;

	  unsigned long nextControlTime;
	  void stateMachine();
	  void track();
	  void mowLanes();
		void mowRandom();
    void printSensorData();
    void readRobotMessages();
    virtual int readSensor(char type);

};

extern RobotClass Robot;

#endif
