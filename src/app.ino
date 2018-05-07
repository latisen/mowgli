/*
    Mainfile for Mowgli 1.0.
    Based off hardware from Liam (project by Jonas Forssell)
    Software is a mix between ardumower and my own written parts.
*/

#include <Arduino.h>
#include "robot.h"
#include "movement.h"
#include "sensing.h"
#include "cutting.h"
#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>

DEFINITION Defaults;
MOVEMENT move(&Robot.perimeter);
SENSING sensing;
CUTTING cutting;
RobotClass Robot;
//Initiera uppkoppling mot esp-link.
ELClient esp(&Serial3, &Serial3);
// CMD klienten
ELClientCmd cmd(&esp);
// MQTT klienten
ELClientMqtt mqtt(&esp);

int state = IDLE;
long time_at_turning = millis();
int turn_direction = 1;
int LCDi = 0;
long templong=millis();
long fiftymsdelay;
long seconddelay;
long hundredmsdelay;
long minutedelay;
long tensdelay;
int slowdowncounter;
long currentRobotMillis;

void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
  }
}

bool connected;

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  mqtt.subscribe("ha/mowgli");
  //mqtt.subscribe("/esp-link/2", 1);
  //mqtt.publish("/esp-link/0", "test1");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial3.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial3.print("data=");
  String data = res->popString();
  Serial.println(data);
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}


void setup() {
    Robot.begin();
    Defaults.definePinsInputOutput();
    fiftymsdelay = 0;
    hundredmsdelay = 0;
    seconddelay = 0;
    slowdowncounter = 0;
    state = IDLE;
    Serial.begin(115200);
    Serial3.begin(115200);
    Serial.println("EL-Client starting!");
    esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
    bool ok;
    do {
        ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
        if (!ok) Serial.println("EL-Client sync failed!");
        } while(!ok);
    Serial.println("EL-Client synced!");
      // Set-up callbacks for events and initialize with es-link.
    mqtt.connectedCb.attach(mqttConnected);
    mqtt.disconnectedCb.attach(mqttDisconnected);
    mqtt.publishedCb.attach(mqttPublished);
    mqtt.dataCb.attach(mqttData);
    mqtt.setup();
    Serial.println("EL-MQTT ready");
}
bool firstRun = true;
static int count;
void loop() {
    currentRobotMillis = millis();
    ADCMan.run();
    esp.Process();
    char buf[12];
    char buf2[12];
    char *test;

    if (millis() >= minutedelay){
      minutedelay = millis() + 60000;

    }

    if(millis() >= tensdelay){
      tensdelay = millis() + 10000;
      if(state == 4){
        mqtt.publish("ha/mowgli/state", "Idle");
      }
      float batFactor = Robot.voltageDividerUges(47, 5.1, 1.0)*Robot.ADC2voltage(1)*10;
      double batADC = ADCMan.read(SOC_PIN);
      float batvolt = (((double)batADC) * batFactor / 10) * 2.355;
      dtostrf(batvolt, 4, 2, buf);
      mqtt.publish("ha/mowgli/battery", buf);
    }
    float batFactor = Robot.voltageDividerUges(47, 5.1, 1.0)*Robot.ADC2voltage(1)*10;
    double batADC = ADCMan.read(SOC_PIN);
    float batvolt = (((double)batADC) * batFactor / 10) * 2.355;
    Serial.println(batvolt);

    /*
    if (connected && (millis()-last) > 4000) {
      Serial.println("publishing");
      t = cmd.GetTime();
      mqtt.publish("ha/mowgli/state", t);
    }
    */

    switch (state) {
      case IDLE:
      Robot.readSensors();
      sensing.getBatteryLevel();
      cutting.startCutting(255);
      if (millis() >= fiftymsdelay){
        Robot.perimeter.getMagnitude(0);
        Robot.perimeter.getMagnitude(1);
        if(Robot.checkInside(1) == false){
          digitalWrite(LED_PIN, HIGH);
        }else{
          digitalWrite(LED_PIN, LOW);
        }
      }
      if(sensing.isInCharger() == true){
        Serial.println("laddar...");
      }
      break;
      case MOWING:
      if(firstRun == true){
        firstRun = false;
        move.GoBackwardUntilInside(0);
      }
      if(batvolt <= 11.00){
        state = FINDPERIM;
      }
      Robot.readSensors();
      //Kör var 50:e millisekund
      if (millis() >= fiftymsdelay){
        fiftymsdelay = millis() + 50;
        Serial.println("LOOP!");
        mqtt.publish("/ha/mowgli/state", 1);
        Robot.perimeter.getMagnitude(0);
        Robot.perimeter.getMagnitude(1);

        if(Robot.perimeter.getMagnitude(0) < -400){
          if(slowdowncounter >= 3){
            move.moveForward(SLOWSPEED, SLOWSPEED);
          }else{
            slowdowncounter++;
          }
        }
        //Sväng höger om sensor 0 är utanför
        if(Robot.checkInside(0) == false){
            move.GoBackwardUntilInside(0);
            move.turnRight(2500);
            slowdowncounter = 0;
            move.moveForward(FULLSPEED_L, FULLSPEED_R);
        }
        //Sväng vänster om sensor 1 är utanför
        if(Robot.checkInside(1) == false){
          move.GoBackwardUntilInside(0);
          move.turnLeft(2500);
          slowdowncounter = 0;
          move.moveForward(FULLSPEED_L, FULLSPEED_R);
        }
      } //Slut på 50ms loopen
      //Kör var 100:e millisekund
      if (millis() >= hundredmsdelay){
        hundredmsdelay = millis() + 100;
        //Generella uppdateringar / avläsningar (ex batterinivå)
      /*  if(sensing.needCharge() == true){
          state = FINDPERIM;
        }*/

      } //Slut på 100ms loopen
      break;

      case LAUNCHING:
        move.moveBackward();
        delay(7000);
        move.turnLeft(2500);
        cutting.startCutting(CUTTERSPEED);
        state = MOWING;
      break;
      case DOCKING:
      if(sensing.isInCharger() == true){
        move.stopMoving();
        state = CHARGING;
      }
      digitalWrite(LED_PIN, HIGH);
      ADCMan.run();
      Robot.perimeter.getMagnitude(0);
      Robot.perimeter.getMagnitude(1);
      if ((Robot.perimeter.isInside(0) != Robot.perimeterInside)){
        Robot.perimeterCounter++;
  			//setSensorTriggered(SEN_PERIM_LEFT);
        Robot.perimeterLastTransitionTime = millis();
        Robot.perimeterInside = Robot.perimeter.isInside(0);
      }

      //PID-tracking av slingtråd
      //Only check efter 30ms
      if (millis() < Robot.nextTimeMotorPerimeterControl) return;

      Robot.perimeterPID.x = 5 * (double(Robot.perimeter.getMagnitude(0)) / 380);

      if (Robot.perimeter.isInside(0)) {
        Robot.perimeterPID.w = -0.5;
      }
      else {
        Robot.perimeterPID.w = 0.5;
      }

      Robot.perimeterPID.y_min = -Robot.MaxSpeedperiPwm ;
      Robot.perimeterPID.y_max = Robot.MaxSpeedperiPwm ;
      Robot.perimeterPID.max_output = Robot.MaxSpeedperiPwm ;
      //and compute
      Robot.perimeterPID.compute();

      if ((millis() > Robot.stateStartTime + 10000) && (millis() > Robot.perimeterLastTransitionTime + Robot.trackingPerimeterTransitionTimeOut)) {

      	//If this condition is true one of the 2 wheels makes backward the other continues with the result of the PID (not advised)
    	  if (Robot.trackingBlockInnerWheelWhilePerimeterStruggling == 0) { //
          if (Robot.perimeterInside) {
            Robot.rightSpeedperi = max(-Robot.MaxSpeedperiPwm, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 2  + Robot.perimeterPID.y));
            Robot.leftSpeedperi = -Robot.MaxSpeedperiPwm / 2;
          }
          else {
            Robot.rightSpeedperi = -Robot.MaxSpeedperiPwm / 2;
            Robot.leftSpeedperi = max(-Robot.MaxSpeedperiPwm, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 2 - Robot.perimeterPID.y));
          }
        }
    	  //If this condition is true one of the 2 wheels stop rotate the other continues with the result of the PID (advised)
        if (Robot.trackingBlockInnerWheelWhilePerimeterStruggling == 1) {
          if (Robot.perimeterInside) {
            Robot.rightSpeedperi = max(-Robot.MaxSpeedperiPwm, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 2  + Robot.perimeterPID.y));
            Robot.leftSpeedperi = 0;
          }
          else {
            Robot.rightSpeedperi = 0;
            Robot.leftSpeedperi = max(-Robot.MaxSpeedperiPwm, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 2 - Robot.perimeterPID.y));
          }
        }

    	  //send to motor
        //setMotorPWM( leftSpeedperi, rightSpeedperi, false);
        move.moveForward(Robot.rightSpeedperi, Robot.leftSpeedperi);
        Serial.println(Robot.perimeter.getMagnitude(0));
        Serial.println(Robot.perimeter.getMagnitude(1));
        //Mower.runForward(100, 100);
        // we record The time at which the last wire loss occurred
        Robot.lastTimeForgetWire = millis();
        // if we have lost the wire from too long time (the robot is running in a circle outside the wire we stop everything)
        if (millis() > Robot.perimeterLastTransitionTime + Robot.trackingErrorTimeOut) {
    				Serial.println(F("Error: tracking error"));
    				//addErrorCounter(ERR_TRACKING);
    				//setNextState(STATE_ERROR,0);
    				//setNextState(STATE_PERI_FIND, 0);
    			}
    			// out of the fonction until the next loop
    			return;
    		}

    		// here we have just found again the wire we need a slow return to let the pid temp react by decreasing its action (perimeterPID.y / PeriCoeffAccel)
    		if ((millis() - Robot.lastTimeForgetWire ) < Robot.trackingPerimeterTransitionTimeOut) {
    			//PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    			Robot.PeriCoeffAccel = (3000.00 - (millis() - Robot.lastTimeForgetWire))/1000.00 ;
    			if (Robot.PeriCoeffAccel < 1.00) Robot.PeriCoeffAccel = 1.00;
    			Robot.rightSpeedperi = max(0, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 1.5 +  Robot.perimeterPID.y / Robot.PeriCoeffAccel));
    			Robot.leftSpeedperi = max(0, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm / 1.5 -  Robot.perimeterPID.y / Robot.PeriCoeffAccel));
    		}
    		else
    		//we are in straight line the pid is total and not/2
    		{
    			Robot.rightSpeedperi = max(0, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm/1.5   + Robot.perimeterPID.y));
    			Robot.leftSpeedperi = max(0, min(Robot.MaxSpeedperiPwm, Robot.MaxSpeedperiPwm/1.5  - Robot.perimeterPID.y));
    		}

    		//setMotorPWM( leftSpeedperi, rightSpeedperi, false);
        move.moveForward(Robot.rightSpeedperi, Robot.leftSpeedperi);
        Serial.println(Robot.perimeter.getMagnitude(0));
        Serial.println(Robot.perimeter.getMagnitude(1));
        //Mower.runForward(100, 100);
    		//if the mower move in perfect straight line the transition between in and out is longuer so you need to reset the perimeterLastTransitionTime

    		if (abs(Robot.perimeterMag ) < Robot.perimeterMagMaxValue/4) {
    			Robot.perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime in perfect sthraith line
    		}

      break;
      case FINDPERIM:
        cutting.stopCutting();
        if(Robot.checkInside(0) == true && Robot.checkInside(1) == true){
          move.moveForward(FULLSPEED_L, FULLSPEED_R);
        }else{
          state = DOCKING;
        }
      break;
      case CHARGING:
        if(sensing.isInCharger() == false){
          move.moveForward(SLOWSPEED, SLOWSPEED);
          delay(1000);
          move.stopMoving();
        }
        if(batvolt >= 13.00){
          state = LAUNCHING;
        }
      break;
      case ERROR:
        move.stopMoving();
        cutting.stopCutting();
      break;
    }

}
