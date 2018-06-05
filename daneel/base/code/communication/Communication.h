/*
 * Communication.h
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

//#ifndef DEBUG_COMM
//#define DEBUG_COMM 0
//#endif

#include <cstdint>
#include <math.h>
#include <vector>
#include <map>
#include <memory>
#include <Arduino.h>
#include <iterator>
#include <HardwareSerial.h>
#include "params.h"

namespace fat{

class Communication {
public:

	struct SpeedCommand{
	public:
		double vx;
		double vy;
		double vtheta;
	};

	struct ActuatorCommand{
		int actuatorId;
		int actuatorCommand;
	};

	struct HMICommand{
		uint8_t redLedCommand;
		uint8_t greenLedCommand;
		uint8_t blueLedCommand;
	};

	struct Repositionning{
		double theta;
	};

	struct SensorCommand{
		int sensorId;
		int sensorCommand;
	};

	typedef void (*SpeedCommandCallback)(SpeedCommand); // Pointer to function, callback types, on per message type
	typedef void (*ActuatorCommandCallback)(ActuatorCommand);
	typedef void (*HMICommandCallback)(HMICommand);
	typedef void (*RepositionningCallback)(Repositionning);
	typedef void (*ResetCallback)(void);
	typedef void (*SensorCommandCallback)(SensorCommand);

	Communication(HardwareSerial serial, uint32_t baudrate);
	virtual ~Communication();

	/**
	 * Functions used to send messages
	 * \return 0: message sent
	 * 1: message sent, but not stored. Won't be resend if ack non recieved...
	 */
	int sendIHMState(const bool cordState, const bool button1State, const bool button2State, const bool redLedState,
			const bool greenLedState, const bool blueLedState);
	int sendActuatorState(const int actuatorId, const int actuatorState);
	int sendOdometryReport(const int dx, const int dy, const double dtheta, const double vx, const double vy, const double vtheta);
	int sendSensorValue(const int sensorId, const int sensorValue);

	/*
	 * Callback registering function. Fonctions registered will be called when the appropriate
	 * message type is recieved and checkMessages is called.
	 * Callback will be called with the appropriate arguments.
	 * \return : int : 0 : callback is registered
	 *                 -1: callback register full, callback not registered
	 */
	int registerSpeedCommandCallback(SpeedCommandCallback callback);
	int registerActuatorCommandCallback(ActuatorCommandCallback callback);
	int registerHMICommandCallback(HMICommandCallback callback);
	int registerRepositionningCallback(RepositionningCallback callback);
	int registerResetCallback(ResetCallback callback);
	int registerSensorCommandCallback(SensorCommandCallback callback);

	void checkMessages();

	void reset();

	unsigned long getTimeSinceLastSpeedMessage() {
		return millis() - timeLastSpeedMessage;
	}

	void setTimeLastSpeedMessage(unsigned long time) {
		timeLastSpeedMessage = time;
	}

private:
	static constexpr int linearOdomToMsgAdder = 32768;
	static constexpr double radianToMsgFactor = 10430.378350470453;
	static constexpr double radianToMsgAdder = M_PI;
	static constexpr double angularSpeedToMsgFactor = 1043.0378350470453;
	static constexpr double angularSpeedToMsgAdder = 10 * M_PI;
	static constexpr int linearSpeedToMsgAdder = 32768;
	static constexpr int upMsgMaxSize = 17;
	static constexpr int upMsgHeaderSize = 3;  // Number of bytes discarded for checksum computation
	static constexpr int downMsgMaxSize = 9;
	static constexpr int downMsgHeaderSize = 3; // Number of bytes discarded for checksum computation
	static constexpr unsigned char hmiCommandRedMask = 7 << 5;
	static constexpr unsigned char hmiCommandGreenMask = 7 << 2;
	static constexpr unsigned char hmiCommandBlueMask = 3;
	static constexpr unsigned int maxCallbackPerMessageType = 10;
	static constexpr unsigned int maxNonAckMessageStored = 50;
	static constexpr unsigned int maxNonAckOdomReportStored = 40;

	//========Start Up Messages definitions======
	typedef enum __attribute__((packed)){
		ACK_DOWN, ODOM_REPORT, HMI_STATE, SENSOR_VALUE
	}eUpMessageType;

	typedef struct __attribute__((packed)) {
		uint8_t ackDownMsgId;
	}sAckDown;
	typedef struct __attribute__((packed)){
		uint8_t previousReportId;
		uint8_t newReportId;
		uint16_t dx;
		uint16_t dy;
		uint16_t dtheta;
		uint16_t vx;
		uint16_t vy;
		uint16_t vtheta;
	}sOdomReportMsg;
	typedef struct __attribute__((packed)){
		uint8_t HMIState;
	}sHMIStateMsg;
	typedef struct __attribute__((packed)){
		uint8_t sensorId;
		uint16_t sensorValue;
	}sSensorValueMsg;
	typedef union __attribute__((packed)){
		sAckDown ackMsg;
		sOdomReportMsg odomReportMsg;
		sHMIStateMsg hmiStateMsg;
		sSensorValueMsg sensorValueMsg;
	}uMessageUpData;

	typedef struct __attribute__((packed)){
		uint8_t upMsgId;
		eUpMessageType upMsgType :8;
		uint8_t checksum;
		uMessageUpData upData;
	}sMessageUp;

	typedef union __attribute__((packed)){
		sMessageUp messageUp;
		char bytes[upMsgMaxSize];
	}uRawMessageUp;

	//========End Up Messages definitions==========
	//========Start Down Messages definitions======
	typedef enum __attribute__((packed)){
		ACK_UP,
		ACK_ODOM_REPORT,
		SPEED_CMD,
		ACTUATOR_CMD,
		HMI_CMD,
		RESET,
		THETA_REPOSITIONING,
		SENSOR_CMD
	}eDownMessageType;
	typedef struct __attribute__((packed)){
		uint8_t ackUpMsgId;
	}sAckUp;
	typedef struct __attribute__((packed)){
		uint8_t ackUpMsgId;
		uint8_t ackOdomReportId;
	}sAckOdomReport;
	typedef struct __attribute__((packed)){
		uint16_t vx;
		uint16_t vy;
		uint16_t vtheta;
	}sSpeedCmd;
	typedef struct __attribute__((packed)){
		uint8_t actuatorId;
		uint16_t actuatorCmd;
	}sActuatorCmd;
	typedef struct __attribute__((packed)){
		uint8_t hmiCmd;
	}sHMICmd;
	typedef struct __attribute__((packed)){
		uint16_t thetaRepositioning;
	}sThetaRepositioning;
	typedef struct __attribute__((packed)){
		uint8_t sensorId;
		uint8_t sensorState;
	}sSensorCmd;
	typedef union __attribute__((packed)){
		sAckUp ackMsg;
		sAckOdomReport ackOdomReportMsg;
		sSpeedCmd speedCmdMsg;
		sActuatorCmd actuatorCmdMsg;
		sHMICmd hmiCmdMsg;
		sThetaRepositioning thetaRepositioningMsg;
		sSensorCmd sensorCmdMsg;
	}uMessageDownData;
	typedef struct __attribute__((packed)){
		uint8_t downMsgId;
		eDownMessageType downMsgType :8;
		uint8_t checksum;
		uMessageDownData downData;
	}sMessageDown;

	typedef union __attribute__((packed)){
		sMessageDown messageDown;
		char bytes[downMsgMaxSize];
	}uRawMessageDown;
	//========End Down Messages definitions==========


	struct sOdomReportStorage{
		uint8_t odomId;
		double dx;
		double dy;
		double dtheta;
	};

	struct sOdomReportBuffer{
		unsigned int startIndex;
		unsigned int writeIndex;
		sOdomReportStorage data[maxNonAckOdomReportStored];
	};

	typedef struct{
		SpeedCommandCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}SpeedMessageCallbackRegister;  // Buffer for callbacks registering, one per message type

	typedef struct{
		ActuatorCommandCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}ActuatorMessageCallbackRegister;

	typedef struct{
		HMICommandCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}HMIMessageCallbackRegister;

	typedef struct{
		RepositionningCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}RepositionningMessageCallbackRegister;

	typedef struct{
		ResetCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}ResetMessageCallbackRegister;

	typedef struct{
		SensorCommandCallback cb[maxCallbackPerMessageType];
		unsigned int index = 0;
	}SensorMessageCallbackRegister;


	/**
	 * sentTime == 0 means this structure is empty. Thus, if valid data is holded by the
	 * struct, sentTime must be different to 0;
	 */
	struct sUpMessageStorage{
		unsigned long sendTime;
		uRawMessageUp message;
	};


	HardwareSerial serial;
	int odomReportIndex;
	int lastOdomReportIndexAcknowledged;
	int upMessageIndex;
	uint8_t lastIdDownMessageRecieved;
	bool isFirstMessage;

	unsigned long timeLastSpeedMessage;

	SpeedMessageCallbackRegister speedMsgCallbacks;
	ActuatorMessageCallbackRegister actuatorMsgCallbacks;
	HMIMessageCallbackRegister HMIMsgCallbacks;
	RepositionningMessageCallbackRegister repositioningCallbacks;
	ResetMessageCallbackRegister resetCallbacks;
	SensorMessageCallbackRegister sensorMsgCallbacks;

	int sendUpMessage(const sMessageUp& msg);
	void recieveMessage(const sMessageDown& msg);
	uint8_t computeUpChecksum(const sMessageUp& msg);
	uint8_t computeDownChecksum(const sMessageDown& msg);
	int storeNewSentMessage(unsigned long time, const uRawMessageUp msg);
	int removeAcknowledgedMessage(uint8_t acknowledgedId);
	sUpMessageStorage toBeAcknowledged[maxNonAckMessageStored];
	sOdomReportBuffer nonAcknowledgedOdomReport;
};

extern Communication communication;

}//namespace fat

#endif /* COMMUNICATION_COMMUNICATION_H_ */
