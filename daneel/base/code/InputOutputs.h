/*
 * InputOuputs.h
 *
 *  Created on: 22 déc. 2017
 *      Author: fabien
 */

#ifndef INPUTOUTPUTS_H_
#define INPUTOUTPUTS_H_

#include <libraries/TM1637/TM1637Display.h>
#include <params.h>

class InputOutputs {
public:
	InputOutputs();
	virtual ~InputOutputs();

	void init();

	void HMISetLedColor(int red, int green, int blue);

	bool HMIGetButton1State();
	bool HMIGetButton2State();

	bool HMIGetCordState();
	void HMISendState();

	void moveArmBase(int degree);
	void moveArmGripper(int degree);

	void handleActuatorMessage(int actuatorId, int actuatorCommand);
	void deliverWater(bool enable);

	void _onNewCordState();
	void _onNewButtonState(int button);

	bool isHmIhasChanged() const {
		return _HMIhasChanged;
	}

	void setHmIhasChanged(bool hmIhasChanged) {
		_HMIhasChanged = hmIhasChanged;
	}

private:
	typedef enum{
		WATER_DELIVERING_DYNAMIXEL = 0,
		WATER_CANNON_DC_MOTOR = 1,
		ARM_BASE_DYNAMIXEL = 2,
		ARM_GRIPPER_DYNAMIXEL = 3,
		SCORE_COUNTER = 4
	}eMsgActuatorId;
	bool _button1Pressed;
	bool _button2Pressed;
	bool _cordIn;
	bool _redLEDOn;
	bool _greenLEDOn;
	bool _blueLEDOn;
	
	volatile bool _HMIhasChanged;

	TM1637Display scoreDisplay;
};

const uint8_t SEG_ENAC[] = {
		SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,  // E
		SEG_E | SEG_G | SEG_C,  // n
		SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
		SEG_A | SEG_F | SEG_E | SEG_D
};

const uint8_t SEG_FAT[] = {
		SEG_A | SEG_F | SEG_E | SEG_G,  // F
		SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
		SEG_A | SEG_B | SEG_C,  // Half T
		SEG_A | SEG_F | SEG_E   // Half T
};

extern InputOutputs inputOutputs;

#endif /* INPUTOUTPUTS_H_ */
