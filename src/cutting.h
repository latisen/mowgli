#include <Arduino.h>

class CUTTING {
    public:
        void startCutting(int setSpeed);
        void stopCutting();

    private:
      bool setupAndDebug=false;
      bool useapi = true;
      int batteryFullLevel=1270;
      int batteryEmptyLevel=1100;
      int batteryGoHomeLevel=1100;
      };
