#include <Arduino.h>
#include "perimeter.h"

class MOVEMENT {
    public:
        MOVEMENT(Perimeter* perimeter);
        void moveForward(int leftSpeed, int rightSpeed);
        void moveBackward();
        void runLeftMotor(int setSpeed, bool setDirection);
        void runRightMotor(int setSpeed, bool setDirection);
        void turnLeft(int turntime);
        void turnRight(int turntime);
        int GoBackwardUntilInside(int inside);
        void stopMoving();
        long currentRobotMillis;
        void blinkLED();


    private:
      int batteryFullLevel=1270;
      int batteryEmptyLevel=1100;
      int batteryGoHomeLevel=1100;
      Perimeter* perimeter;
      };
