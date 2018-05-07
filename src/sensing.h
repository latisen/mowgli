#include <Arduino.h>

class SENSING {
    public:
        bool obstacleFront();
        int getBatteryLevel();
        bool needCharge();
        bool isInCharger();
        bool isFullyCharged();

    private:
      bool setupAndDebug=false;
      bool useapi = true;
      int batteryFullLevel=1270;
      int batteryEmptyLevel=1100;
      int batteryGoHomeLevel=1100;
      };
