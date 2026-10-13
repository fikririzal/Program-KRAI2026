/*
 * Joystick.c
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

//Joystick
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Swerve.h"
#include "USART.h"
#include "myDEF.h"
#include "math.h"
#include "Joystick.h"
#include "joystickRizal.h"
#include "cmsis_os2.h"
#include "Profiler.h"

float speed = minSpeed, steer = minSteer;

void Joystick(void) {
	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, speed, -steer);
		else
			Kinematik(0, speed, 0);
		//Led2On;
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, -speed, -steer);
		else
			Kinematik(0, -speed, 0);
		//Led2Off;
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(speed, 0, -steer);
		else
			Kinematik(speed, 0, 0);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(-speed, 0, -steer);
		else
			Kinematik(-speed, 0, 0);
	} else if (PS_R1 != 0) {
		Kinematik(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		Kinematik(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickKiriSamping(void) {
	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, speed, -steer);
		else
			Kinematik(0, speed + 10, 0);
		//Led2On;
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, -speed, -steer);
		else
			Kinematik(0, -speed, 0);
		//Led2Off;
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(speed, 0, -steer);
		else
			Kinematik(speed, 0, 0);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(-speed, 0, -steer);
		else
			Kinematik(-speed - 8, 7, 0);
	} else if (PS_R1 != 0) {
		Kinematik(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		Kinematik(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

float targetH;
void JoystickHeadingLock30DegV3(void) {

	float ErrorH, Vh;
	float SpeedAdd;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, Vh);
		//Led2On;
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, Vh);
		//Led2Off;
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, Vh);
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, Vh);
	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLock30DegV3HadapKanan(void) {

	float ErrorH, Vh;
	float SpeedAdd;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, Vh);

		//Led2On;
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, Vh);

		//Led2Off;
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, Vh);

	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLock30DegV3PosKanan(void) {

	float ErrorH, Vh;
	float SpeedAdd;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, Vh);

	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, Vh);

	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, Vh);

	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLock30DegNembakManual(void) {

	float ErrorH, Vh;
	float SpeedAdd;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {

		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, Vh);

	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, Vh);

	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, Vh);

	} else if (PS_Right != 0) {

		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30DegV2(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void Joystick30Deg(void) {

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.9f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, 0);
		//Led2On;
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, 0);
		//Led2Off;
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, 0);
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, 0);
	} else if (PS_R1 != 0) {
		Kinematik30DegV2(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		Kinematik30DegV2(0, 0, (steer + 1) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

//
//
//float targetH;
//void JoystickHeadingLock(uint8_t Mode)
//{
//	float ErrorH, Vh;
//	float SpeedAdd, SpeedAddSter = 30.0;
//
//	if(Mode==FastMode)SpeedAdd=5.0;
//	else if(Mode==SlowMode)SpeedAdd=5.0;
//
////	if(PS_Up!=0||PS_Down!=0||PS_Right!=0||PS_Left!=0||PS_R1!=0||PS_L1!=0)
////	{
////		ErrorH = targetH - RobotYaw;
//////		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//////		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
////		if		  	(ErrorH>180) ErrorH = ErrorH - 360;
////		else if		(ErrorH<-180)	ErrorH = ErrorH +360;
////		SetBatas(&ErrorH , 5);
////		Vh = 1*ErrorH;
////		//Vh = 0;
////	}
//
////	else
////	{
////		targetH = RobotYaw;
////	}
//
//
//	if(PS_R1!=0 || PS_L1!=0) 	{if(steer<maxSteer) steer+=0.25f;}//0.15
//	else											steer = minSteer;
//
//	if(PS_LYPlus!=0)
//	{		if 			(PS_R1!=0)	Kinematik(0,speed+SpeedAdd,steer);
//			else if (PS_L1!=0)	Kinematik(0,speed+SpeedAdd,-steer);
//			else										Kinematik(0,speed+SpeedAdd,Vh);
//	}
//	else if(PS_LYMin!=0)
//	{		if 			(PS_R1!=0)	Kinematik(0,-speed-SpeedAdd,steer);
//			else if (PS_L1!=0)	Kinematik(0,-speed-SpeedAdd,-steer);
//			else										Kinematik(0,-speed-SpeedAdd,Vh);
//	}
//	else if(PS_LXPlus!=0)
//	{		if 			(PS_R1!=0)	Kinematik(speed+SpeedAdd,0,steer);
//			else if (PS_L1!=0)	Kinematik(speed+SpeedAdd,0,-steer);
//			else										Kinematik(speed+SpeedAdd,0,Vh);
//	}
//	else if(PS_LXMin!=0)
//	{	if 			(PS_R1!=0)	Kinematik(-speed-SpeedAdd,0,steer);
//		else if (PS_L1!=0)	Kinematik(-speed-SpeedAdd,0,-steer);
//		else										Kinematik(-speed-SpeedAdd,0,Vh);
//	}
//	else if(PS_R1!=0)
//	{
//		targetH = RobotYaw;
//		if(Mode == SlowMode)
//		{
//			Kinematik(0,0,SpeedAddSter+(steer+1)*1.5f);
//		}else Kinematik(0,0,(steer+1)*1.5f);
//	}
//	else if(PS_L1!=0)
//	{
//		targetH = RobotYaw;
//	if(Mode == SlowMode)
//		{
//			Kinematik(0,0,-SpeedAddSter+(steer+1)*-1.5f);
//		}else Kinematik(0,0,(steer+1)*-1.5f);
//	}
//
//	else
//	{
//		speed=minSpeed;
//		steer=minSteer;
//		RodaStop();
//
//	}
//
//	if(speed<maxSpeed) speed+=0.9f;
//	HAL_Delay(10);
//}

void JoystickHeadingLockRIZAL(uint8_t Mode) {
	/*
	 * Static local state:
	 * Tidak global, tapi nilainya tetap tersimpan antar pemanggilan function.
	 */
	static float targetH = 0.0f;
	static float manualSpeed = 0.0f;
	static float manualSteer = 0.0f;
	static uint8_t initialized = 0U;

	/*
	 * Tuning parameter.
	 * Semua dibuat const local supaya mudah tuning di satu tempat,
	 * tanpa #define global.
	 */
	const float slowMinSpeed = 10.0f;
	const float slowMaxSpeed = 50.0f;
	const float slowAccel = 0.20f;

	const float slowRotBase = 4.0f;
	const float slowRotMax = 15.0f;
	const float slowRotAccel = 0.10f;

	const float slowHeadingKp = 0.7f;
	const float slowHeadingLimit = 3.0f;

	const float fastMinSpeed = 25.0f;
	const float fastMaxSpeed = 180.0f;
	const float fastAccel = 0.8f;

	const float fastRotBase = 30.0f;
	const float fastRotMax = 70.0f;
	const float fastRotAccel = 0.30f;

	const float fastHeadingKp = 1.0f;
	const float fastHeadingLimit = 5.0f;

	float minSpeedMode;
	float maxSpeedMode;
	float accelSpeed;

	float rotBase;
	float rotMax;
	float rotAccel;

	float headingKp;
	float headingLimit;

	float Vx = 0.0f;
	float Vy = 0.0f;
	float omega = 0.0f;

	uint8_t moveInput;
	uint8_t rotateInput;

	if (initialized == 0U) {
		targetH = RobotYaw;
		manualSpeed = slowMinSpeed;
		manualSteer = 0.0f;
		initialized = 1U;
	}

	/*
	 * Pilih profile berdasarkan mode.
	 * Final mapping:
	 * L2/FastMode = normal speed
	 * R2/SlowMode = precision speed
	 */
	if (Mode == FastMode) {
		minSpeedMode = fastMinSpeed;
		maxSpeedMode = fastMaxSpeed;
		accelSpeed = fastAccel;

		rotBase = fastRotBase;
		rotMax = fastRotMax;
		rotAccel = fastRotAccel;

		headingKp = fastHeadingKp;
		headingLimit = fastHeadingLimit;
	} else {
		minSpeedMode = slowMinSpeed;
		maxSpeedMode = slowMaxSpeed;
		accelSpeed = slowAccel;

		rotBase = slowRotBase;
		rotMax = slowRotMax;
		rotAccel = slowRotAccel;

		headingKp = slowHeadingKp;
		headingLimit = slowHeadingLimit;
	}

	moveInput = (DPAD_ATAS != 0) || (DPAD_BAWAH != 0) || (DPAD_KANAN != 0)
			|| (DPAD_KIRI != 0);

	rotateInput = (PS_R1 != 0) || (PS_L1 != 0);

	/*
	 * Tidak ada input:
	 * reset ramp, update target heading ke yaw sekarang, lalu stop.
	 */
	if ((moveInput == 0U) && (rotateInput == 0U)) {
		manualSpeed = minSpeedMode;
		manualSteer = 0.0f;
		targetH = RobotYaw;

		RodaStop();
		osDelay(5);
		return;
	}

	/*
	 * Ramp translasi.
	 * SlowMode dan FastMode sekarang punya max speed masing-masing.
	 */
	if (manualSpeed < maxSpeedMode) {
		manualSpeed += accelSpeed;

		if (manualSpeed > maxSpeedMode) {
			manualSpeed = maxSpeedMode;
		}
	} else if (manualSpeed > maxSpeedMode) {
		/*
		 * Kalau mode berubah dari Fast ke Slow saat tombol masih ditekan,
		 * speed langsung dibatasi ke max Slow.
		 */
		manualSpeed = maxSpeedMode;
	}

	/*
	 * Ramp rotasi manual.
	 */
	if (rotateInput != 0U) {
		if (manualSteer < rotMax) {
			manualSteer += rotAccel;

			if (manualSteer > rotMax) {
				manualSteer = rotMax;
			}
		}
	} else {
		manualSteer = 0.0f;
	}

	/*
	 * Translasi dari DPAD.
	 */
	if (DPAD_ATAS != 0) {
		Vy += manualSpeed;
	}

	if (DPAD_BAWAH != 0) {
		Vy -= manualSpeed;
	}

	if (DPAD_KANAN != 0) {
		Vx += manualSpeed;
	}

	if (DPAD_KIRI != 0) {
		Vx -= manualSpeed;
	}

	/*
	 * Diagonal compensation.
	 * Lebih aman daripada mengurangi fixed -20,
	 * karena tidak akan membalik arah saat speed kecil.
	 */
	if ((Vx != 0.0f) && (Vy != 0.0f)) {
		Vx *= 0.707f;
		Vy *= 0.707f;
	}

	/*
	 * Rotasi manual R1/L1.
	 * Saat user rotate manual, target heading di-reset ke RobotYaw
	 * supaya setelah dilepas robot tidak balik ke heading lama.
	 */
	if (rotateInput != 0U) {
		targetH = RobotYaw;

		if (PS_R1 != 0) {
			omega = rotBase + manualSteer;
		} else if (PS_L1 != 0) {
			omega = -rotBase - manualSteer;
		}
	} else {
		/*
		 * Heading lock saat translasi tanpa R1/L1.
		 */
		float errorH = targetH - RobotYaw;

		if (errorH > 180.0f) {
			errorH -= 360.0f;
		} else if (errorH < -180.0f) {
			errorH += 360.0f;
		}

		SetBatas(&errorH, headingLimit);

		omega = headingKp * errorH;
	}

	/*
	 * Output tetap sama seperti function lama.
	 */
	Kinematik(Vx, Vy, omega);

	osDelay(5);
}

float targetH;
void JoystickHeadingLock(uint8_t Mode) {
	float ErrorH = 0, Vh = 0;
	float SpeedAdd = 5.0f, SpeedAddSter = 40.0f;
	float buf_stering = 30;
	float Vx = 0, Vy = 0;  // Untuk arah X (kanan/kiri) dan Y (maju/mundur)

	if (Mode == FastMode)
		SpeedAdd = 30.0f;
	else if (Mode == SlowMode) {
		SpeedAddSter = 4.0f;
		SpeedAdd = -20.0f;
	}

//    else if (Mode == SlowMode) SpeedAdd = 5.0f;

	if (DPAD_ATAS || DPAD_BAWAH || DPAD_KANAN || DPAD_KIRI || LY != 0 || LX != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
//		Vh = 0;
	}

	else {
		targetH = RobotYaw;
	}

	// Penyesuaian steer jika sedang rotasi manual
	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.25f;
	} else {
		steer = minSteer;
	}

//    // Hitung komponen arah
//    if (LY < 0) Vy += speed + SpeedAdd;
//    if (LY > 0)  Vy -= speed + SpeedAdd;
//    if (LX > 0) Vx += speed + SpeedAdd;
//    if (LX < 0)  Vx -= speed + SpeedAdd;

	if (DPAD_ATAS != 0)
		Vy += speed + SpeedAdd;
	if (DPAD_BAWAH != 0)
		Vy -= speed + SpeedAdd;
	if (DPAD_KANAN != 0)
		Vx += speed + SpeedAdd;
	if (DPAD_KIRI != 0)
		Vx -= speed + SpeedAdd;

	if (Vx != 0 && Vy != 0) {
		if (Vx > 0)
			Vx -= 20;
		else if (Vx < 0)
			Vx += 20;
		if (Vy > 0)
			Vy -= 20;
		else if (Vy < 0)
			Vy += 20;
	}
	// Jika ada input gerak
	if (Vx != 0 || Vy != 0) {
		if (PS_R1 != 0)
			Kinematik(Vx, Vy, buf_stering + steer);
		else if (PS_L1 != 0)
			Kinematik(Vx, Vy, -steer - buf_stering);
		else
			Kinematik(Vx, Vy, Vh);
	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		if (Mode == FastMode)
			Kinematik(0, 0, SpeedAddSter + (steer + 1.0f) * 1.5f);
		else
			Kinematik(0, 0, SpeedAddSter + (steer + 1.0f) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		if (Mode == FastMode)
			Kinematik(0, 0, -SpeedAddSter + (steer + 1.0f) * -1.5f);
		else
			Kinematik(0, 0, -SpeedAddSter + (steer + 1.0f) * -1.5f);
	} else {
		speed = minSpeed;
		steer = minSteer;
		if (Mode == FastMode) {
			RodaStop();
		}
//        else if(Mode == Kunci )
//        {
//         Parkir();
//        }
		else {
			RodaStop();
		}

	}

	if (speed < maxSpeed)
		speed += 0.9f;

	osDelay(5); // Rizal
}

float targetH;
void JoystickHeadingLock30DegRingKedua(void) {
	float ErrorH, Vh;
	float SpeedAdd;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 1 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.5f;
	} else
		steer = minSteer;

	if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, speed, -steer);
		else
			Kinematik30DegV2(0, speed, Vh);
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(0, -speed, -steer);
		else
			Kinematik30DegV2(0, -speed, Vh);
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(speed, 0, -steer);
		else
			Kinematik30DegV2(speed, 0, Vh);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30DegV2(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30DegV2(-speed, 0, -steer);
		else
			Kinematik30DegV2(-speed, 0, Vh);
	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * -1.5f);
	}

	else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}

	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLock30Deg(void) {
	float ErrorH, Vh;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 0.9 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.5f;
	} else
		steer = minSteer;

	if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(0, -speed, -steer);
		else
			Kinematik30Deg(0, -speed, Vh);
	} else if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(0, speed, -steer);
		else
			Kinematik30Deg(0, speed, Vh);

	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(-speed, 0, -steer);
		else
			Kinematik30Deg(-speed, 0, Vh);
	} else if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(speed, 0, -steer);
		else
			Kinematik30Deg(speed, 0, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * -1.5f);
	}

	else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}

	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLock30DegInverse(void) {
	float ErrorH, Vh;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 0.9 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.5f;
	} else
		steer = minSteer;

	if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(0, speed, -steer);
		else
			Kinematik30Deg(0, speed, Vh);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(0, -speed, -steer);
		else
			Kinematik30Deg(0, -speed, Vh);
	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(speed, 0, -steer);
		else
			Kinematik30Deg(speed, 0, Vh);
	} else if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik30Deg(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik30Deg(-speed, 0, -steer);
		else
			Kinematik30Deg(-speed, 0, Vh);
	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik30Deg(0, 0, (steer + 1) * -1.5f);
	}

	else {
		speed = minSpeed;
		steer = minSteer;
		RodaStop();
	}

	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLockKinematikInverse(uint8_t Trace) {
	float ErrorH, Vh;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 0.9 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.5f;
	} else
		steer = minSteer;

	if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, -speed, -steer);
		else
			Kinematik(0, -speed, Vh);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, speed, -steer);
		else
			Kinematik(0, speed, Vh);

	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(-speed, 0, -steer);
		else
			Kinematik(-speed, 0, Vh);
	} else if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(speed, 0, -steer);
		else
			Kinematik(speed, 0, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik(0, 0, (steer + 1) * -1.5f);
	}

	else {
		speed = minSpeed;
		steer = minSteer;
		//Kinematik(0,18,0);
		//RodaStop();
		if (Trace == TraceDepan)
			Kinematik(-20, 0, 0);
		else if (Trace == TraceBelakang)
			Kinematik(20, 0, 0);
		else if (Trace == TraceKanan)
			Kinematik(0, 20, 0);
		else if (Trace == TraceKiri)
			Kinematik(0, -20, 0);
		else if (Trace == NoneTrace)
			RodaStop();
		else if (Trace == TraceParkir)
			KinematikSpeedOffParkir(0, 0, 2);
////
	}

	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

void JoystickHeadingLockKinematikTraceGrip(uint8_t Trace) {
	float ErrorH, Vh;

	if (PS_Up != 0 || PS_Down != 0 || PS_Right != 0 || PS_Left != 0
			|| PS_R1 != 0 || PS_L1 != 0) {
		ErrorH = targetH - RobotYaw;
//		sprintf(pesan,"%.1f %.1f %.1f\n", ErrorH, RobotYaw, targetH);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;
		SetBatas(&ErrorH, 5);
		Vh = 0.9 * ErrorH;
	}

	else {
		targetH = RobotYaw;
	}

	if (PS_R1 != 0 || PS_L1 != 0) {
		if (steer < maxSteer)
			steer += 0.5f;
	} else
		steer = minSteer;

	if (PS_Right != 0) {
		if (PS_R1 != 0)
			Kinematik(speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(speed, 0, -steer);
		else
			Kinematik(speed, 0, Vh);
	} else if (PS_Left != 0) {
		if (PS_R1 != 0)
			Kinematik(-speed, 0, steer);
		else if (PS_L1 != 0)
			Kinematik(-speed, 0, -steer);
		else
			Kinematik(-speed, 0, Vh);

	} else if (PS_Down != 0) {
		if (PS_R1 != 0)
			Kinematik(0, -speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, -speed, -steer);
		else
			Kinematik(0, -speed, Vh);

	} else if (PS_Up != 0) {
		if (PS_R1 != 0)
			Kinematik(0, speed, steer);
		else if (PS_L1 != 0)
			Kinematik(0, speed, -steer);
		else
			Kinematik(0, speed, Vh);

	} else if (PS_R1 != 0) {
		targetH = RobotYaw;
		Kinematik(0, 0, (steer + 1) * 1.5f);
	} else if (PS_L1 != 0) {
		targetH = RobotYaw;
		Kinematik(0, 0, (steer + 1) * -1.5f);
	}

	else {
		speed = minSpeed;
		steer = minSteer;
		//Kinematik(0,18,0);
		//RodaStop();
		if (Trace == TraceDepan)
			Kinematik(-20, 0, 0);
		else if (Trace == TraceBelakang)
			Kinematik(20, 0, 0);
		else if (Trace == TraceKanan)
			Kinematik(0, 20, 0);
		else if (Trace == TraceKiri)
			Kinematik(0, -20, 0);
		else if (Trace == NoneTrace)
			RodaStop();
////
	}
	if (speed < maxSpeed)
		speed += 0.9f;
	HAL_Delay(10);
}

#define KINEMATIK_MAX_SPEED_RIZAL 127.0f

static float AbsFloat_RIZAL(float x) {
	return (x < 0.0f) ? -x : x;
}

static float LimitFloat_RIZAL(float x, float min, float max) {
	if (x > max)
		return max;
	if (x < min)
		return min;
	return x;
}

static float NormalizeHeadingError_RIZAL(float error) {
	if (error > 180.0f)
		error -= 360.0f;
	else if (error < -180.0f)
		error += 360.0f;

	return error;
}

/*
 * Accel only:
 * - Kalau target lebih besar dari command sekarang, naik perlahan.
 * - Kalau target lebih kecil, langsung ikut target.
 * - Kalau tombol dilepas, langsung 0.
 * - Kalau arah dibalik, langsung ikut target.
 */
static float AccelOnly_RIZAL(float now, float target, float step) {
	if (target == 0.0f) {
		return 0.0f;
	}

	if ((now > 0.0f && target < 0.0f) || (now < 0.0f && target > 0.0f)) {
		return target;
	}

	if (AbsFloat_RIZAL(target) <= AbsFloat_RIZAL(now)) {
		return target;
	}

	if (target > now) {
		now += step;
		if (now > target)
			now = target;
	} else if (target < now) {
		now -= step;
		if (now < target)
			now = target;
	}

	return now;
}

/*
 * Limiter input Kinematik.
 * Tujuannya supaya Vx, Vy, dan omega yang masuk ke Kinematik tidak membuat
 * speed roda melebihi 127.
 *
 * Ini mengikuti rumus di function Kinematik().
 */
static void LimitInputKinematik_RIZAL(float *Vx, float *Vy, float *omega_deg) {
	float omega = (*omega_deg) / 180.0f * pi;

	float V1x = (omega * r) + (*Vx);
	float V1y = (*Vy);

	float V2x = (-omega * r * Sin30) + (*Vx);
	float V2y = (-omega * r * Sin60) + (*Vy);

	float V3x = (-omega * r * Sin30) + (*Vx);
	float V3y = (omega * r * Sin60) + (*Vy);

	float s1 = sqrtf((V1x * V1x) + (V1y * V1y));
	float s2 = sqrtf((V2x * V2x) + (V2y * V2y));
	float s3 = sqrtf((V3x * V3x) + (V3y * V3y));

	float max_s = s1;

	if (s2 > max_s)
		max_s = s2;
	if (s3 > max_s)
		max_s = s3;

	if (max_s > KINEMATIK_MAX_SPEED_RIZAL) {
		float scale = KINEMATIK_MAX_SPEED_RIZAL / max_s;

		*Vx *= scale;
		*Vy *= scale;
		*omega_deg *= scale;
	}
}

void JoystickHeadingLock_RIZAL(uint8_t Mode) {
	PROF_BEGIN(PROF_JOYSTICK_RIZAL);
	static float cmdVx = 0.0f;
	static float cmdVy = 0.0f;
	static float cmdW = 0.0f;
	static uint8_t lastMode = 255;

	float targetVx = 0.0f;
	float targetVy = 0.0f;
	float targetW = 0.0f;

	float maxMove;
	float accelMove;

	float maxRot;
	float accelRot;

	float headingLimit;
	float headingKp;

	float ErrorH = 0.0f;
	float Vh = 0.0f;

	uint8_t moveInput;
	uint8_t rotateInput;

	/*
	 * =====================================================
	 * //Kecepatam
	 * =====================================================
	 *
	 * maxMove   = batas translasi Vx/Vy
	 * accelMove = akselerasi translasi per 5 ms
	 *
	 * maxRot    = batas omega_deg
	 * accelRot  = akselerasi rotasi per 5 ms
	 *
	 * FastMode:
	 *   responsif
	 *
	 * SlowMode / SuperSlowMode:
	 *   presisi, naik speed pelan, tapi saat lepas tombol langsung stop
	 */

	if (Mode == FastMode) {
		maxMove = 127.0f;
		accelMove = 3.0f;

		maxRot = 70.0f;
		accelRot = 10.0f;

		headingLimit = 5.0f;
		headingKp = 1.0f;
	} else if (Mode == SlowMode) {
		maxMove = 70.0f;
		accelMove = 20.00f;

		maxRot = 50.0f;
		accelRot = 20.0f;

		headingLimit = 5.0f;
		headingKp = 1.0f;
	} else {
		maxMove = 127.0f;
		accelMove = 2.0f;

		maxRot = 70.0f;
		accelRot = 1.5f;

		headingLimit = 5.0f;
		headingKp = 1.0f;
	}

	/*
	 * Kalau pindah mode, command langsung dibatasi ke mode baru.
	 * Ini penting saat pindah FastMode ke SlowMode.
	 */
	if (lastMode != Mode) {
		cmdVx = LimitFloat_RIZAL(cmdVx, -maxMove, maxMove);
		cmdVy = LimitFloat_RIZAL(cmdVy, -maxMove, maxMove);
		cmdW = LimitFloat_RIZAL(cmdW, -maxRot, maxRot);
		lastMode = Mode;
	}

	moveInput = (DPAD_ATAS != 0) || (DPAD_BAWAH != 0) || (DPAD_KANAN != 0)
			|| (DPAD_KIRI != 0);

	rotateInput = (PS_R1 != 0) || (PS_L1 != 0);

	/*
	 * =====================================================
	 * Heading lock
	 * =====================================================
	 */

	if (moveInput || rotateInput || LY != 0 || LX != 0) {
		ErrorH = targetH - RobotYaw;
		ErrorH = NormalizeHeadingError_RIZAL(ErrorH);
		ErrorH = LimitFloat_RIZAL(ErrorH, -headingLimit, headingLimit);

		Vh = headingKp * ErrorH;
	} else {
		targetH = RobotYaw;
		Vh = 0.0f;
	}

	/*
	 * Saat rotasi manual, target heading dibuat yaw saat ini.
	 * Supaya ketika R1/L1 dilepas, robot tidak balik ke heading lama.
	 */
	if (rotateInput) {
		targetH = RobotYaw;
	}

	/*
	 * =====================================================
	 * Target translasi
	 * =====================================================
	 */

	if (DPAD_ATAS != 0)
		targetVy += maxMove;
	if (DPAD_BAWAH != 0)
		targetVy -= maxMove;
	if (DPAD_KANAN != 0)
		targetVx += maxMove;
	if (DPAD_KIRI != 0)
		targetVx -= maxMove;

	/*
	 * Diagonal dinormalisasi supaya resultan tidak melebihi maxMove.
	 */
	if (targetVx != 0.0f && targetVy != 0.0f) {
		targetVx *= 0.707f;
		targetVy *= 0.707f;
	}

	/*
	 * =====================================================
	 * Target rotasi
	 * =====================================================
	 */

	if (PS_R1 != 0) {
		targetW = maxRot;
	} else if (PS_L1 != 0) {
		targetW = -maxRot;
	} else {
		targetW = Vh;
	}

	/*
	 * =====================================================
	 * Acceleration only
	 * =====================================================
	 *
	 * Tidak ada deceleration ramp.
	 * Kalau tombol dilepas, command langsung 0.
	 */

	cmdVx = AccelOnly_RIZAL(cmdVx, targetVx, accelMove);
	cmdVy = AccelOnly_RIZAL(cmdVy, targetVy, accelMove);
	cmdW = AccelOnly_RIZAL(cmdW, targetW, accelRot);

	/*
	 * Deadband kecil.
	 */
	if (AbsFloat_RIZAL(cmdVx) < 0.5f)
		cmdVx = 0.0f;
	if (AbsFloat_RIZAL(cmdVy) < 0.5f)
		cmdVy = 0.0f;
	if (AbsFloat_RIZAL(cmdW) < 0.3f)
		cmdW = 0.0f;

	/*
	 * Limit sesuai Kinematik.
	 * Ini mencegah speed roda hasil inverse kinematic lewat 127.
	 */
//    LimitInputKinematik_RIZAL(&cmdVx, &cmdVy, &cmdW);
	/*
	 * =====================================================
	 * Output
	 * =====================================================
	 */

	if (cmdVx != 0.0f || cmdVy != 0.0f || cmdW != 0.0f) {
		Kinematik(cmdVx, cmdVy, cmdW);
	} else {
		RodaStop();
	}

	osDelay(5);
	PROF_END(PROF_JOYSTICK_RIZAL);
}
