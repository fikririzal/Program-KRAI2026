/*
 * Joystick.h
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "main.h"
#include "myDEF.h"

//Program Jalan Robot
extern void Joystick(void);
extern void JoystickHeadingLock(uint8_t Mode);
extern void JoystickHeadingLockTrace(uint8_t PilihTrace,float SpeedTrace);
extern void JoystickAnalog();
extern void JoystickHeadingLock30Deg();
extern void Joystick30Deg(void);
extern void JoystickHeadingLock30DegV3(void);
extern void JoystickHeadingLock30DegV3HadapKanan(void);
extern void JoystickHeadingLock30DegV3PosKanan(void);
extern void JoystickKiriSamping(void);
extern void JoystickHeadingLock30DegNembakManual(void);

extern void JoystickHeadingLock30DegRingKedua(void);
extern void JoystickHeadingLock30DegInverse(void);
extern void JoystickHeadingLockKinematikInverse(uint8_t Trace);
extern void ToolTurret(uint8_t IndexPos);
extern void RobotToolTembak();
extern void RobotToolGripper();
extern void JoystickHeadingLockKinematikTraceGrip(uint8_t Trace);
void JoystickHeadingLock_RIZAL(uint8_t Mode);

extern uint8_t FlagJoystick;
extern uint8_t IndexTurret;
extern short SudutYawTurret;
extern short SudutPitchTurret;
extern short SpeedL[];
extern short SpeedR[];
extern float PosPitchTurret[15];
extern short PosYawTurret[15];
extern uint8_t FlagIndexPos;

#endif /* INC_JOYSTICK_H_ */
