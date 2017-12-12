/*
 * BaseHolonomy.cpp
 *
 *  Created on: 16 nov. 2017
 *      Author: fabien
 */

#include <Arduino.h>
#include <Odometry.h>
#include "params.h"
#include "utilities.h"
//#include <DynamixelSerial5.h>


Odometry odometry = Odometry();

Odometry::Odometry() {
	_readIndex = 0;
	_thetaAI = 0;
	_inc1 = _inc2 = _inc3 = 0;
	_speed = makeSpeed(0,0,0);
	_motorSpeeds = makeSpeed(0,0,0);
	float32_t* m_data = (float32_t*)malloc(3*sizeof(float32_t));
	m_data[0] = m_data[1] = m_data[2] = 0;
}

Odometry::~Odometry() {
}

Move3D Odometry::getMoveDelta(int originId) {
	Move3D move;
	int currentIndex = _readIndex;

	//TODO correct potential bug : if originId < lastId - MOVE_HISTORY_LENGHT, it will loop forever !
	//Add a counter or whatever to protect against this bug.

	//sum the moves from id originId to the most recent (in reverse order because it's easier)
	do {
		move = move + _moveDelta[currentIndex];		//TODO : use += operator
		currentIndex--;
	} while(_moveDelta[currentIndex].getId()>originId);

	return move;
}

float Odometry::getDeltaTheta() {
	return _moveDelta[_readIndex].getTheta();
}

void Odometry::update() {
#if defined(TRIKE)
	updateTrike();
#elif defined(DIFFERENTIAL)
	updateDifferential();
#elif defined(HOLONOMIC)
	updateHolonomic();
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif
}

void Odometry::init() {
}

void Odometry::updateTrike() {

}

void Odometry::updateDifferential() {
	//TODO (or not)
}

void Odometry::updateHolonomic() {
	cli();				//disable interrupts to copy and reset the counters
	int inc1 = _inc1;
	int inc2 = _inc2;
	int inc3 = _inc3;
	_inc1 = 0;
	_inc2 = 0;
	_inc3 = 0;
	sei();				//enable interrupts

	//tangential distance traveled by each wheel
	_motorSpeeds->pData[0] = (float)inc1 / INC_PER_MM / CONTROL_PERIOD;
	_motorSpeeds->pData[1] = (float)inc2 / INC_PER_MM / CONTROL_PERIOD;
	_motorSpeeds->pData[2] = (float)inc3 / INC_PER_MM / CONTROL_PERIOD;

	arm_matrix_instance_f32* currentSpeed = makeSpeed(0,0,0);

	arm_status status = arm_mat_mult_f32(&Dplus, _motorSpeeds, currentSpeed);
	if(status != ARM_MATH_SUCCESS) {
		Serial.print("[ERROR] updateHolonomic: matrix multiplication error : ");
		Serial.println(status);
	}

	Serial.print(inc1);
	Serial.print(";");
//	Serial.print(inc2);
//	Serial.print(";");
//	Serial.println(inc3);
//	Serial.print(";");
//	Serial.print(inc2);
//	Serial.print("\t");
//	Serial.println(inc3);

//	Serial.print("x=");
//	Serial.print(v.pData[0]);
//	Serial.print("\ty=");
//	Serial.print(v.pData[1]);
//	Serial.print("\tR*theta=");
//	Serial.println(v.pData[2]/ROBOT_RADIUS);

	Move3D move = Move3D(currentSpeed->pData[0], currentSpeed->pData[1], currentSpeed->pData[2]/ROBOT_RADIUS);
	addMove(move);

	//speed is created by malloc as currentSpeed, then passed to _speed, then free.
	freeSpeed(_speed);
	_speed = currentSpeed;



}

void Odometry::addMove(Move3D move) {
	float theta = _thetaAI + getDeltaTheta();

	//change reference system
	float x = move.getX() * cos(theta) - move.getY() * sin(theta);
	float y = move.getY() * cos(theta) + move.getX() * sin(theta);

	//modify the object rather than create a new one
	move.setX(x);
	move.setY(y);

	//Add this move to the other
	_moveDelta[_readIndex] = _moveDelta[_readIndex] + move;	//TODO Use += operator


//		Serial.print("x=");
//		Serial.print(_moveDelta[_readIndex].getX());
//		Serial.print("\ty=");
//		Serial.print(_moveDelta[_readIndex].getY());
//		Serial.print("\ttheta=");
//		Serial.println(_moveDelta[_readIndex].getTheta());
}

void initOdometry() {
#if defined(TRIKE)
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
#elif defined(DIFFERENTIAL)
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);
#elif defined(HOLONOMIC)
	pinMode(MOT1_ENCA, INPUT);
	pinMode(MOT1_ENCB, INPUT);
	pinMode(MOT2_ENCA, INPUT);
	pinMode(MOT2_ENCB, INPUT);
	pinMode(MOT3_ENCA, INPUT);
	pinMode(MOT3_ENCB, INPUT);
	attachInterrupt(MOT1_ENCA, ISR1, RISING);
	attachInterrupt(MOT1_ENCB, ISR11, RISING);
	attachInterrupt(MOT2_ENCA, ISR2, RISING);
	attachInterrupt(MOT2_ENCB, ISR22, RISING);
	attachInterrupt(MOT3_ENCA, ISR3, RISING);
	attachInterrupt(MOT3_ENCB, ISR33, RISING);
#else
#error "No Robot type defined (TRIKE, DIFFERENTIAL or HOLONOMIC)"
#endif

	odometry.init();
}

void ISR1() {
	odometry.isr1();
}

void ISR11() {
	odometry.isr11();
}

void ISR2() {
	odometry.isr2();
}

void ISR22() {
	odometry.isr22();
}

void ISR3() {
	odometry.isr3();
}

void ISR33() {
	odometry.isr33();
}

void Odometry::isr1() {
	if(digitalRead(MOT1_ENCB)) {
		_inc1--;
	} else {
		_inc1++;
	}
}

void Odometry::isr11() {
	if(digitalRead(MOT1_ENCA)) {
		_inc1++;
	} else {
		_inc1--;
	}
}

void Odometry::isr2() {
	if(digitalRead(MOT2_ENCB)) {
		_inc2--;
	} else {
		_inc2++;
	}
}

void Odometry::isr22() {
	if(digitalRead(MOT2_ENCA)) {
		_inc2++;
	} else {
		_inc2--;
	}
}

void Odometry::isr3() {
	if(digitalRead(MOT3_ENCB)) {
		_inc3--;
	} else {
		_inc3++;
	}
}

void Odometry::isr33() {
	if(digitalRead(MOT3_ENCA)) {
		_inc3++;
	} else {
		_inc3--;
	}
}
