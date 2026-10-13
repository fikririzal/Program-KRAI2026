/*
 * dyn.h
 *
 *  Created on: Jul 1, 2026
 *      Author: USER
 */

#ifndef INC_DYN_H_
#define INC_DYN_H_

//#include "myDEF.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern void MoveServo(int servoID, int Position);
extern void MoveDrive(int servoID, int Speed);
extern void EnableTorque(int servoID, uint8_t Enable);
extern void SetAngleLimitCCW(int servoID, short Angle);
extern void SetServoKp(int servoID, uint8_t Kp);
extern void SetServoTorqueLimit(int servoID, int Torque);
extern void SetSpeed(int servoID, uint16_t Speed);

#endif /* INC_DYN_H_ */
