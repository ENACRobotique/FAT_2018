/*
 * params.h
 *
 *  Created on: 23 févr. 2018
 *      Author: fabien
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "Arduino.h"

#define SERVO1 A0
#define SERVO2 A1
#define SERVO3 A2
#define MOSFET_UL 5
#define MOSFET_BL 6
#define MOSFET_UR 11
#define MOSFET_BR 10

#define SUN_PIN MOSFET_UR
#define F_PIN MOSFET_BL
#define A_PIN MOSFET_UL
#define T_PIN MOSFET_BR

#define STEPPER_DIR 8
#define STEPPER_STEP 9
#define STEPPER_ENABLE 7

#define TRAIN_PERIOD 3
#define FAT_PERIOD 1000
#define SUN_PERIO 8

#define SUN_INC 1
#define MIN_SUN_US 900
#define MAX_SUN_US 2250
#define FAT_CLOCK_PERIOD 8



#endif /* PARAMS_H_ */
