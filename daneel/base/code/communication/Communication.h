/*
 * Communication.h
 *
 *  Created on: 4 déc. 2017
 *      Author: guilhem
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

#include <cstdint>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <Arduino.h>
#include <HardwareSerial.h>

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
		bool redLedCommand;
		bool greenLedCommand;
		bool blueLedCommand;
	};

	struct Repositionning{
		double theta;
	};

	Communication(HardwareSerial serial, uint32_t baudrate);
	virtual ~Communication();

	int sendIHMState(const bool cordState, const bool button1State, const bool button2State, const bool redLedState,
			const bool greenLedState, const bool blueLedState);
	int sendActuatorState(const int actuatorId, const int actuatorState);
	int sendOdometryReport(const int dx, const int dy, const double dtheta);

	void registerRecieveSpeedCommandCallback(std::function<void (const SpeedCommand&)>);
	void registerRecieveActuatorCommandCallback(std::function<void (const ActuatorCommand&)>);
	void registerRecieveHMICommandCallback(std::function<void (const HMICommand&)>);

	void checkMessages();

private:
	static constexpr int linearOdomToMsgAdder = 32768;
	static constexpr double radianToMsgFactor = 10430.378350470453;
	static constexpr double radianToMsgAdder = M_PI;
	static constexpr int upMsgMaxSize = 11;
	static constexpr int upMsgHeaderSize = 3;  // Number of bytes discarded for checksum computation
	static constexpr int downMsgMaxSize = 9;
	static constexpr int downMsgHeaderSize = 3; // Number of bytes discarded for checksum computation
	static constexpr unsigned char hmiCommandRedMask = 1 << 7;
	static constexpr unsigned char hmiCommandGreenMask = 1 << 6;
	static constexpr unsigned char hmiCommandBlueMask = 1 << 5;

	//========Start Up Messages definitions======
	typedef enum __attribute__((packed)){
		ACK_UP, ODOM_REPORT, HMI_STATE, ACTUATOR_STATE
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
	}sOdomReportMsg;
	typedef struct __attribute__((packed)){
		uint8_t HMIState;
	}sHMIStateMsg;
	typedef struct __attribute__((packed)){
		uint8_t actuatorId;
		uint16_t actuatorValue;
	}sActuatorStateMsg;
	typedef union __attribute__((packed)){
		sAckDown ackMsg;
		sOdomReportMsg odomReportMsg;
		sHMIStateMsg hmiStateMsg;
		sActuatorStateMsg actuatorStateMsg;
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
		ACK_DOWN,
		ACK_ODOM_REPORT,
		SPEED_CMD,
		ACTUATOR_CMD,
		HMI_CMD
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
	typedef union __attribute__((packed)){
		sAckUp ackMsg;
		sAckOdomReport ackOdomReportMsg;
		sSpeedCmd speedCmdMsg;
		sActuatorCmd actuatorCmdMsg;
		sHMICmd hmiCmdMsg;
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

	HardwareSerial serial;
	int odomReportIndex;
	int lastOdomReportIndexAcknowledged;
	int upMessageIndex;
	int cumulateddx;
	int cumulateddy;
	double cumulateddtheta;
	uint8_t lastIdDownMessageRecieved;
	bool isFirstMessage;

	//Todo : Maybe an "unordered_map<downMsgType, vector<function<void (const DownMessage&)>>>
	// callbacks; With DownMessage super class of all user exposed messages data (SpeedCommand, ...)
	std::vector<std::function<void (const SpeedCommand&)> > speedMsgCallbacks;
	std::vector<std::function<void (const ActuatorCommand&)> > actuatorMsgCallbacks;
	std::vector<std::function<void (const HMICommand&)> > HMIMsgCallbacks;
	std::vector<std::function<void (const Repositionning&)> > repositioningCallbacks;

	int sendUpMessage(const sMessageUp& msg);
	void recieveMessage(const sMessageDown& msg);
	uint8_t computeUpChecksum(const sMessageUp& msg);
	uint8_t computeDownChecksum(const sMessageDown& msg);
	std::map<const unsigned long, uRawMessageUp> toBeAcknowledged;
	std::vector<sOdomReportMsg> nonAcknowledgedOdomReport;
};
}//namespace fat

#endif /* COMMUNICATION_COMMUNICATION_H_ */
