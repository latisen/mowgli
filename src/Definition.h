#include <Arduino.h>

#ifndef _DEFINITION_H_
#define _DEFINITION_H_

#define IOREF 5.0

//Motorhastigheter
#define FULLSPEED_R 100
#define FULLSPEED_L 100
#define SLOWSPEED 30
#define CUTTERSPEED 100

// Analoga pins
#define WHEEL_MOTOR_A_CURRENT_PIN A0
#define WHEEL_MOTOR_B_CURRENT_PIN A1
#define SOC_PIN A2
#define pinBatteryVoltage A2
#define CUTTER_CURRENT_PIN A3
#define I2C_SDA_PIN 4
#define I2C_SDL_PIN 5
#define pinPerimeterLeft A8
#define pinPerimeterRight A9
#define pinVoltageMeasurement A7

// Digitala pins
#define RX_PIN 0
#define TX_PIN 1
#define BWF_SENSOR_INPUT_PIN 2
#define WHEEL_MOTOR_A_PWM_PIN 3
#define BWF_SELECT_A_PIN 4
#define DOCK_PIN 5
#define CUTTER_PWM_PIN 6
#define BWF_SELECT_B_PIN 7
#define BUMPER 8
#define LIFT_SENSOR_PIN 9
#define LED_PIN 10
#define WHEEL_MOTOR_B_PWM_PIN 11
#define WHEEL_MOTOR_A_DIRECTION_PIN 12
#define WHEEL_MOTOR_B_DIRECTION_PIN 13

// Inställningar hjulmotorer
#define WHEELMOTOR_OVERLOAD		130
#define WHEELMOTOR_SMOOTHNESS	300
#define OVERLOAD_INTERVAL 3000
#define TURNTIME = 2000;

// Inställningar klippmotor
#define CUTTER_OVERLOAD 100

// De states som används
const int MOWING = 0;
const int LAUNCHING = 1;
const int DOCKING = 2;
const int CHARGING = 3;
const int IDLE = 4;
const int FINDPERIM = 5;
const int ERROR = 6;

/* Software version */
#define MAJOR_VERSION 0
#define MINOR_VERSION_1	1
#define MINOR_VERSION_2	0

class DEFINITION {
    public:
        void definePinsInputOutput();
        long currentRobotMillis;

    private:
      bool setupAndDebug=false;
      bool useapi = true;
      int batteryFullLevel=1270;
      int batteryEmptyLevel=1100;
      int batteryGoHomeLevel=1100;
      };

#endif /* _DEFINITION_H_ */
