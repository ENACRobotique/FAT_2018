/*
 * Communication.cpp
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#include "Communication.h"

using namespace std;
namespace fat {

Communication communication = Communication(RASPI_COMMUNICATION_SERIAL, RASPI_COMMUNICATION_BAUDRATE);


Communication::Communication(HardwareSerial serial, uint32_t baudrate):serial(serial), odomReportIndex(0),
		lastOdomReportIndexAcknowledged(0),upMessageIndex(0), lastIdDownMessageRecieved(0),
		isFirstMessage(true){
	this->serial.begin(baudrate);
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		toBeAcknowledged[i].sendTime = 0;
	}
	nonAcknowledgedOdomReport.startIndex = 0;
	nonAcknowledgedOdomReport.writeIndex = 0;
	timeLastSpeedMessage = 0;

}

Communication::~Communication() {
	// TODO Auto-generated destructor stub
}

int Communication::sendIHMState(const bool cordState, const bool button1State, const bool button2State,
		const bool redLedState, const bool greenLedState, const bool blueLedState){
	sMessageUp msg;
	uint8_t hmiState = cordState << 7 | button1State << 6 | button2State << 5 | redLedState << 4 | greenLedState << 3 |
			blueLedState << 2;
	msg.upMsgType = Communication::HMI_STATE;
	msg.upData.hmiStateMsg.HMIState = hmiState;

	return sendUpMessage(msg);
}

int Communication::sendSensorValue(const int sensorId, const int sensorValue){
	sMessageUp msg;
	msg.upMsgType = Communication::SENSOR_VALUE;
	if (sensorId < 0 || sensorId > 255){
		return -10;
	}
	if (sensorValue < 0 || sensorValue > 65535){
		return -11;
	}
	msg.upData.sensorValueMsg.sensorId = sensorId;
	msg.upData.sensorValueMsg.sensorValue = sensorValue;

	return sendUpMessage(msg);
}

int Communication::sendOdometryReport(const int dx, const int dy, const double dtheta, const double vx,
		const double vy, const double vtheta){
	sMessageUp msg;
	sOdomReportStorage odomReport;
	int cumuleddx = dx, cumuleddy = dy;
	double cumuleddtheta = dtheta;

	odomReportIndex = (odomReportIndex + 1) % 256;
	odomReport = {(uint8_t)odomReportIndex, (double)dx, (double)dy, (double)dtheta};

	for (unsigned int i = 0; i < (nonAcknowledgedOdomReport.writeIndex -
		nonAcknowledgedOdomReport.startIndex + maxNonAckOdomReportStored) % maxNonAckOdomReportStored; i++){
		// we must have lastOdomAck < odomId < odomReportIndex to add it in the report (but maybe check if the mod. is correct)
		cumuleddx += nonAcknowledgedOdomReport.data[(nonAcknowledgedOdomReport.startIndex + i) % maxNonAckOdomReportStored].dx;
		cumuleddy += nonAcknowledgedOdomReport.data[(nonAcknowledgedOdomReport.startIndex + i) % maxNonAckOdomReportStored].dy;
		cumuleddtheta += nonAcknowledgedOdomReport.data[(nonAcknowledgedOdomReport.startIndex + i) % maxNonAckOdomReportStored].dtheta;
	}

	nonAcknowledgedOdomReport.data[nonAcknowledgedOdomReport.writeIndex] = odomReport;
	nonAcknowledgedOdomReport.writeIndex = (nonAcknowledgedOdomReport.writeIndex + 1) % maxNonAckOdomReportStored;

	int msgdx = cumuleddx + linearOdomToMsgAdder;
	int msgdy = cumuleddy + linearOdomToMsgAdder;
	int msgdtheta = round((cumuleddtheta + radianToMsgAdder) * radianToMsgFactor);
	int msgvx = vx + linearSpeedToMsgAdder;
	int msgvy = vy + linearSpeedToMsgAdder;
	int msgvtheta = round((vtheta + angularSpeedToMsgAdder) * angularSpeedToMsgFactor);

	if (msgdx < 0 || msgdx > 65535){
		return -10;
	}
	if (msgdy < 0 || msgdy > 65535){
		return -11;
	}
	if (msgdtheta < 0 || msgdtheta > 65535){
		return -12;
	}
	msg.upMsgType = Communication::ODOM_REPORT;
	msg.upData.odomReportMsg.previousReportId = lastOdomReportIndexAcknowledged;
	msg.upData.odomReportMsg.newReportId = odomReportIndex;
	msg.upData.odomReportMsg.dx = msgdx;
	msg.upData.odomReportMsg.dy = msgdy;
	msg.upData.odomReportMsg.dtheta = msgdtheta;
	msg.upData.odomReportMsg.vx = msgvx;
	msg.upData.odomReportMsg.vy = msgvy;
	msg.upData.odomReportMsg.vtheta = msgvtheta;

#if DEBUG_COMM
	Serial.print("Sending Odom Report : from (");
	Serial.print(lastOdomReportIndexAcknowledged);
	Serial.print("; ");
	Serial.print(odomReportIndex);
	Serial.print("; ");
	Serial.print(cumuleddx);
	Serial.print("; ");
	Serial.print(cumuleddy);
	Serial.print("; ");
	Serial.print(cumuleddtheta);
	Serial.print("; ");
	Serial.print(vx);
	Serial.print("; ");
	Serial.print(vy);
	Serial.print("; ");
	Serial.print(vtheta);
#endif

	return sendUpMessage(msg);
}

int Communication::storeNewSentMessage(unsigned long time, const uRawMessageUp msg){
	bool stored = false;
	sUpMessageStorage toStore = {time, msg};
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		if (toBeAcknowledged[i].sendTime == 0){
			toBeAcknowledged[i] = toStore;
			stored = true;
			break;
		}
	}
	if (stored){
		return 0;
	}
	return 1;
}

int Communication::removeAcknowledgedMessage(uint8_t acknowledgedId){
	bool isRemoved = false;
	for (unsigned int i=0; i < maxNonAckMessageStored; i++){
		if (toBeAcknowledged[i].message.messageUp.upMsgId == acknowledgedId){
			toBeAcknowledged[i].sendTime = 0;
			isRemoved = true;
		}
	}
	return isRemoved;
}

int Communication::sendUpMessage(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	int storageSuccess;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	rawMessage.messageUp.checksum = computeUpChecksum(msg);
	rawMessage.messageUp.upMsgId = upMessageIndex;
	upMessageIndex++;
	serial.write(rawMessage.bytes, upMsgMaxSize);
	storageSuccess = storeNewSentMessage(millis(), rawMessage);
	// Todo : Handle resend on timeout if ack is not recieved -> Delete stored message on ack recieve and update send time on resend.
	return storageSuccess;
}

uint8_t Communication::computeUpChecksum(const sMessageUp& msg){
	uRawMessageUp rawMessage;
	unsigned char checksum = 0;
	for (int i = 0; i < upMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageUp = msg;
	for (int i = upMsgHeaderSize; i < upMsgMaxSize; i++){
		checksum = (checksum ^ rawMessage.bytes[i]) % 256;
	}
	return checksum;
}

uint8_t Communication::computeDownChecksum(const sMessageDown& msg){
	uRawMessageDown rawMessage;
	unsigned char checksum = 0;
	for (int i = 0; i < downMsgMaxSize; i++){
		rawMessage.bytes[i] = 0;
	}
	rawMessage.messageDown = msg;
	for (int i = downMsgHeaderSize; i < downMsgMaxSize; i++){
		checksum = (checksum ^ rawMessage.bytes[i]) % 256;
	}
	return checksum;
}

int Communication::registerActuatorCommandCallback(ActuatorCommandCallback callback){
	if (actuatorMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	actuatorMsgCallbacks.cb[actuatorMsgCallbacks.index] = callback;
	actuatorMsgCallbacks.index++;
	return 0;
}

int Communication::registerHMICommandCallback(HMICommandCallback callback){
	if (HMIMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	HMIMsgCallbacks.cb[HMIMsgCallbacks.index] = callback;
	HMIMsgCallbacks.index++;
	return 0;
}

int Communication::registerSpeedCommandCallback(SpeedCommandCallback callback){
	if (speedMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	speedMsgCallbacks.cb[speedMsgCallbacks.index] = callback;
	speedMsgCallbacks.index++;
	return 0;
}

int Communication::registerRepositionningCallback(RepositionningCallback callback){
	if (repositioningCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	repositioningCallbacks.cb[repositioningCallbacks.index] = callback;
	repositioningCallbacks.index++;
	return 0;
}

int Communication::registerResetCallback(ResetCallback callback){
	if (resetCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	resetCallbacks.cb[resetCallbacks.index] = callback;
	resetCallbacks.index++;
	return 0;
}

int Communication::registerSensorCommandCallback(SensorCommandCallback callback){
	if (sensorMsgCallbacks.index >= maxCallbackPerMessageType){
		return -1;
	}
	sensorMsgCallbacks.cb[sensorMsgCallbacks.index] = callback;
	sensorMsgCallbacks.index++;
	return 0;
}

void Communication::recieveMessage(const sMessageDown& msg){
	vector<sOdomReportStorage>::iterator nonAckOdomReportItr;
	SpeedCommand speedCommand;
	ActuatorCommand actuatorCommand;
	HMICommand hmiCommand;
	Repositionning thetaRepositioning;
	SensorCommand sensorCommand;

	switch(msg.downMsgType){
	case ACK_UP:
		removeAcknowledgedMessage(msg.downData.ackMsg.ackUpMsgId);
		break;
	case ACK_ODOM_REPORT:
#if DEBUG_COMM
		Serial.print("New Ack Odom : ");
#endif
		removeAcknowledgedMessage(msg.downData.ackOdomReportMsg.ackUpMsgId);

		unsigned int index;
		for (unsigned int i = 0; i < (nonAcknowledgedOdomReport.writeIndex -
				nonAcknowledgedOdomReport.startIndex + maxNonAckOdomReportStored) % maxNonAckOdomReportStored; i++){
			index = (nonAcknowledgedOdomReport.startIndex + i) % maxNonAckOdomReportStored;
			//Not sure... We want to delete "all" the stored id that are < acknowledged received one.
#if DEBUG_COMM
			Serial.print("Difference in odom indices : ");
			Serial.println((msg.downData.ackOdomReportMsg.ackOdomReportId - nonAcknowledgedOdomReport.data[index].odomId + 256) % 256);
#endif
			if ((msg.downData.ackOdomReportMsg.ackOdomReportId - nonAcknowledgedOdomReport.data[index].odomId + 256) % 256 == 	0 ){
				nonAcknowledgedOdomReport.startIndex = index + 1;
#if DEBUG_COMM
			Serial.print("Wanted new index found : ");
			Serial.println(index);
#endif
				break;
			}
		}
		lastOdomReportIndexAcknowledged = msg.downData.ackOdomReportMsg.ackOdomReportId;

#if DEBUG_COMM
		Serial.print("AckOdom : ");
		Serial.print(msg.downData.ackOdomReportMsg.ackOdomReportId);
		Serial.print(" new index : ");
		Serial.println(nonAcknowledgedOdomReport.startIndex);
#endif

		break;
	case SPEED_CMD:
		speedCommand.vx = msg.downData.speedCmdMsg.vx - linearSpeedToMsgAdder;  // Todo : Maybe scale linear speed
 		speedCommand.vy = msg.downData.speedCmdMsg.vy - linearSpeedToMsgAdder;
 		speedCommand.vtheta = msg.downData.speedCmdMsg.vtheta / angularSpeedToMsgFactor - angularSpeedToMsgAdder;
 		for (unsigned int i=0; i < speedMsgCallbacks.index; i++){
			speedMsgCallbacks.cb[i](speedCommand);
		}
		break;
	case ACTUATOR_CMD:
		actuatorCommand.actuatorId = msg.downData.actuatorCmdMsg.actuatorId;
		actuatorCommand.actuatorCommand = msg.downData.actuatorCmdMsg.actuatorCmd;
		for (unsigned int i=0; i < actuatorMsgCallbacks.index; i++){
			actuatorMsgCallbacks.cb[i](actuatorCommand);
		}
		break;
	case HMI_CMD:
		hmiCommand.redLedCommand = (msg.downData.hmiCmdMsg.hmiCmd & hmiCommandRedMask) >> 5 / 7 * 255;
		hmiCommand.greenLedCommand = (msg.downData.hmiCmdMsg.hmiCmd & hmiCommandGreenMask) >> 2 / 7 * 255;
		hmiCommand.blueLedCommand = (msg.downData.hmiCmdMsg.hmiCmd & hmiCommandBlueMask) / 7 * 255;
		for (unsigned int i=0; i < HMIMsgCallbacks.index; i++){
			HMIMsgCallbacks.cb[i](hmiCommand);
		}
		break;
	case RESET:
		for (unsigned int i = 0; i < resetCallbacks.index; i++){
			resetCallbacks.cb[i]();
		}
		break;
	case THETA_REPOSITIONING:
		thetaRepositioning.theta = msg.downData.thetaRepositioningMsg.thetaRepositioning / radianToMsgFactor - radianToMsgAdder;
		for (unsigned int i = 0; i < (nonAcknowledgedOdomReport.writeIndex -
			nonAcknowledgedOdomReport.startIndex + maxNonAckOdomReportStored) % maxNonAckOdomReportStored; i++){
			// Adds all the non acknowledged odometry report. They cannot have been taken into account by the AI.
			thetaRepositioning.theta += nonAcknowledgedOdomReport.data[(nonAcknowledgedOdomReport.startIndex + i) % maxNonAckOdomReportStored].dtheta;
		}
		for (unsigned int i = 0; i < repositioningCallbacks.index; i++){
			repositioningCallbacks.cb[i](thetaRepositioning);
		}
		break;
	case SENSOR_CMD:
		sensorCommand.sensorId = msg.downData.sensorCmdMsg.sensorId;
		sensorCommand.sensorCommand = msg.downData.sensorCmdMsg.sensorState;
		for (unsigned int i=0; i < sensorMsgCallbacks.index; i++){
			sensorMsgCallbacks.cb[i](sensorCommand);
		}
		break;
	}
}

void Communication::checkMessages(){
	uRawMessageDown rawDataDown;
	uRawMessageUp rawUpAckMessage;
	if (serial.available()) { //If there is some data waiting in the buffer
		if (serial.available() >= downMsgMaxSize) { //Read all the data in the buffer (asserting raspi is sending at max one message per teensy loop)
			for (int i = 0; i < downMsgMaxSize; i++){
				rawDataDown.bytes[i] = serial.read();
			}
#if DEBUG_COMM
			char tmp[16];
			for (int debug = 0; debug < downMsgMaxSize; debug++){
				sprintf(tmp, "%.2X",rawDataDown.bytes[debug]);
				Serial.print(tmp);
			}
			Serial.println();
#endif
			if (computeDownChecksum(rawDataDown.messageDown) == rawDataDown.messageDown.checksum){
				rawUpAckMessage.messageUp.upMsgType = ACK_DOWN;
				rawUpAckMessage.messageUp.upData.ackMsg.ackDownMsgId = rawDataDown.messageDown.downMsgId;
				serial.write(rawUpAckMessage.bytes, upMsgMaxSize);


				if (isFirstMessage || //If it is the first message, accept it
						rawDataDown.messageDown.downMsgType == RESET ||
						((rawDataDown.messageDown.downMsgId - lastIdDownMessageRecieved + 256 )%256>0
								&& (rawDataDown.messageDown.downMsgId - lastIdDownMessageRecieved)%256<128)) { //Check if the message has a id bigger than the last recevied
					isFirstMessage = false;
					lastIdDownMessageRecieved = rawDataDown.messageDown.downMsgId;

					recieveMessage(rawDataDown.messageDown);

				}
			}/* else {
				raw_ack_message.msg.type = NON_ACK;
				raw_ack_message.msg.down_id = raw_data_down.msg.id;
				HWSERIAL.write(raw_ack_message.data, MSG_UP_MAX_SIZE);
				return 0;*/

		}
	} /*else {
		return 0; //Serial is empty :'(
	}*/
}

void Communication::reset(){
	for (unsigned int i = 0; i < maxNonAckMessageStored; i++){
		toBeAcknowledged[i].sendTime = 0;
	}
	nonAcknowledgedOdomReport.startIndex = 0;
	nonAcknowledgedOdomReport.writeIndex = 0;
	odomReportIndex = 0;
	lastOdomReportIndexAcknowledged = 0;
	upMessageIndex = 0;
	lastIdDownMessageRecieved = 0;
	isFirstMessage = true;
	while(serial.available()){
		serial.read();
	}
}

} /* namespace fat */
