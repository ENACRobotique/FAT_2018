// Do not remove the include below
#include "propulsion_FAT_2018.h"
#include <Arduino.h>
#include "communication/Communication.h"
#include "Odometry.h"
#include "MotorControl.h"
#include "Metro.h"
#include "params.h"
#include "utilities.h"
#include "ExtNavigation.h"
#include "InputOutputs.h"
//#include "DynamixelSerial5.h"
#ifdef SIMULATOR
#include "Simulator.h"
#endif

using namespace fat;
bool blink;
unsigned long blinkTime;

void handleLEDCallback(const Communication::HMICommand msg);
void testHMICallback(const Communication::HMICommand msg);  // Forward declaration
void testActuatorCallback(const Communication::ActuatorCommand msg);
void setNewTableSpeedCallback(const Communication::SpeedCommand msg);
void handleActuatorCallback(const Communication::ActuatorCommand msg);
void resetCallback();
void handleRepositionningCallback(const Communication::Repositionning msg);
void handleSensorCommandCallback(const Communication::SensorCommand cmd);


Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD*1000));
Metro posReportTme = Metro((unsigned long)(POS_REPORT_PERIOD*1000));
Metro IOsTime = Metro((unsigned long)(IO_REPORT_PERIOD*1000));
Metro BallDetectorTime = Metro((unsigned long)(100));

//Metro testTime = Metro(4000);
//
//float32_t testCommands[][3] = {
//		{0, 0, 0.5},
//		{0, 0, 0},
//		{0, 0, -0.3},
//		{0, 0, 0}
//};
int i = 0;

void setup()
{
	//pinMode(25, OUTPUT);
	//digitalWrite(25, HIGH);
	initOdometry();
	motorControl.init();
	inputOutputs.init();
	Serial.begin(115200);
	//while(!Serial.available());
	Serial.println("Start");
	Serial.flush();
//	testTime.reset();
	controlTime.reset();
	
	blinkTime = millis();
	blink = false;
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, blink);
	communication.registerHMICommandCallback(testHMICallback);
	communication.registerHMICommandCallback(handleLEDCallback);
	communication.registerActuatorCommandCallback(testActuatorCallback);
	communication.registerActuatorCommandCallback(handleActuatorCallback);
	communication.registerSpeedCommandCallback(setNewTableSpeedCallback);
	communication.registerResetCallback(resetCallback);
	communication.registerRepositionningCallback(handleRepositionningCallback);
	communication.registerSensorCommandCallback(handleSensorCommandCallback);
}


void loop()
{
	if(millis() - blinkTime >= 1500){
		blink ^= 1;
		digitalWrite(LED_PIN, blink);
		blinkTime = millis();
	}
	communication.checkMessages();

	if(controlTime.check()) {
#ifdef SIMULATOR
		simulator.update();
#endif
		odometry.update();
		extNavigation.update();
		motorControl.control();

	}

	if(posReportTme.check()) {
		communication.sendOdometryReport(odometry.getMoveDelta()->pData[0], odometry.getMoveDelta()->pData[1],
				odometry.getMoveDelta()->pData[2], odometry.getSpeed()->pData[0], odometry.getSpeed()->pData[1],
				odometry.getSpeed()->pData[2]);
		odometry.resetMoveDelta();
	}

	if(IOsTime.check()) {
		inputOutputs.run();
	}

	if (BallDetectorTime.check()){
		inputOutputs.updateBallDetector();
	}

//	if(testTime.check()) {
//		//motorControl.setTargetSpeed(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
//		extNavigation.setTableSpeedCons(testCommands[i][0], testCommands[i][1], testCommands[i][2]);
//		i = (i+1) % 4;
//	}

}

void handleLEDCallback(const Communication::HMICommand msg){
	inputOutputs.HMISetLedColor(msg.redLedCommand, msg.greenLedCommand, msg.blueLedCommand);
}

void testHMICallback(const Communication::HMICommand msg){
	Serial.print("Force rouge : ");
	Serial.print(msg.redLedCommand);
	Serial.print("\tForce verte : ");
	Serial.print(msg.greenLedCommand);
	Serial.print("\tForce bleue : ");
	Serial.println(msg.blueLedCommand);
}

void testActuatorCallback(const Communication::ActuatorCommand msg){
	Serial.print("Actuator Id : ");
	Serial.print(msg.actuatorId);
	Serial.print("\tActuator command : ");
	Serial.println(msg.actuatorCommand);
}

void handleActuatorCallback(const Communication::ActuatorCommand msg){
	inputOutputs.handleActuatorMessage(msg.actuatorId, msg.actuatorCommand);
	//Dynamixel.ledStatus(msg.actuatorId, msg.actuatorCommand);
}

void setNewTableSpeedCallback(const Communication::SpeedCommand msg){
	fat::communication.setTimeLastSpeedMessage(millis());
	extNavigation.setTableSpeedCons(msg.vx, msg.vy, msg.vtheta);
}

void resetCallback(){
	motorControl.reset();
	odometry.reset();
	extNavigation.reset();
#ifdef SIMULATOR
	simulator.reset();
#endif
	communication.reset();
	inputOutputs.reset();
}

void handleRepositionningCallback(const Communication::Repositionning msg){
	odometry.recalerTheta(msg.theta);
	Serial.print("Received (with non ack added) : ");
	Serial.println(msg.theta);
	Serial.print("Set theta to : ");
	Serial.println(odometry.getTheta());
}

void handleSensorCommandCallback(Communication::SensorCommand cmd){
	inputOutputs.handleSensorCommand(cmd.sensorId, cmd.sensorCommand);
}
