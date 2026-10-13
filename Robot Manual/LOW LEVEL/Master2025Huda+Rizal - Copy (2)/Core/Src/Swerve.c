/*
 * Swerve.c
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

//#include "myDEF.h"
#include "math.h"
#include "Swerve.h"
#include "USART.h"
#include "Profiler.h"
#include "stdio.h"

extern float R1_DBG_ErrorX;
extern float R1_DBG_ErrorY;
extern float R1_DBG_ErrorJ;
extern float R1_DBG_ErrorH;
extern float R1_DBG_Sudut;
extern float R1_DBG_Vx;
extern float R1_DBG_Vy;
extern float R1_DBG_Omega;

extern float R1_DBG_PosX_Robot;
extern float R1_DBG_PosY_Robot;
extern float R1_DBG_PosX_World;
extern float R1_DBG_PosY_World;
extern float R1_DBG_RobotYaw;
extern int16_t R1_DBG_SpeedEncA;
extern int16_t R1_DBG_SpeedEncB;

extern volatile float DataYawIMU;

extern volatile uint8_t PS_INT;

void loopDelay(unsigned short ddd) {
	uint32_t DDD, x;
	float a = 0.1;

	for (x = 0; x < ddd; x++)
		for (DDD = 0; DDD < 1000000; DDD++) {
			a *= (float) DDD;
		}
//	while(--ddd>=0)
//	{
//     for(DDD=0;DDD<1000000;DDD++);		//loop sekitar 1 ms
//	}
}

float moving_window_break[2][5];

//inline void custom_delay_s(short delay)
//{
//	uint32_t y;
//	y = delayManual+(delay/5);
//	while(delayManual<=y);
//}

void custom_delay_s(short delay) {
	if (FlagPaksa == 0) {
		HAL_Delay(delay);
	}
}

_Bool PaksaR = 0;
_Bool FlagPaksa = 0;
_Bool FlagPaksaTrace = 0;
volatile _Bool SequenceTurret = 0;
volatile _Bool DoneReloadRing = 0;

_Bool flag_puter_roda = 0;

void SetBatas(float *x, float batas) {
	if (*x > batas)
		*x = batas;		// jika melebihi batas maksimal, diset ke nilai maksimal
	else if (*x < -batas)
		*x = -batas;// jika kurang dari batas minimal, diset ke batas minimal
}

// mengembalikan nilai mutlak dari tipe bilangan float
float absf(float x) {
	if (x < 0)
		return -x;
	else
		return x;
}

void hitungSudutSpeed(float sudutAwal, float sudutBaru, roda *R) {
	float x;
	x = sudutBaru - sudutAwal;
	if (x > 90) {
		if (x > 270) {
			R->sudut = sudutBaru;
		} else {
			x = x - 180 + sudutAwal;
			if (x > 180)
				x -= 360;
			else if (x < -180)
				x += 360;
			R->sudut = x;
			R->speed = -R->speed;
		}
	} else if (x < -90) {
		if (x < -270) {
			R->sudut = sudutBaru;
		} else {
			x = x + 180 + sudutAwal;
			if (x > 180)
				x -= 360;
			else if (x < -180)
				x += 360;
			R->sudut = x;
			R->speed = -R->speed;
		}
	} else {
		R->sudut = sudutBaru;
	}
}

void Kinematik30DegV2(float Vx, float Vy, float omega_deg) {
	roda X;

////	//Kanan
//	int16_t VX  = (Vy * Sin30)	+ 	(Vx * Cos30);
//	int16_t VY = (Vy * Cos30)	+ (-1*(Vx * Sin30));
//
	//Kiri
	int16_t VX = (-1 * (Vy * Sin30)) + (1 * (Vx * Cos30));
	int16_t VY = (1 * (Vy * Cos30)) + ((Vx * Sin30));

	//	uint8_t pesan[50];
	Vx = VX;
	Vy = VY;
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

//	// hitng sudut roda relatif pada body robot
//	float sudut_M1 = -(atan2(V1y, V1x))*180/pi+30;
//	float sudut_M2 = -(atan2(V2y, V2x))*180/pi-120+30;
//	float sudut_M3 = -(atan2(V3y, V3x))*180/pi+120+30;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);
	//HAL_Delay(5);
	X.speed = speed_M2;

	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);
	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
}

void Kinematik30Deg(float Vx, float Vy, float omega_deg) {
	roda X;

//	//Kanan
	int16_t VX = (Vy * Sin30) + (Vx * Cos30);
	int16_t VY = (Vy * Cos30) + (-1 * (Vx * Sin30));

	//Kiri
//	int16_t VX  = (-1*(Vy * Sin30))	+ 	(1*(Vx * Cos30));
//	int16_t VY = (1*(Vy * Cos30))	+ ((Vx * Sin30));

	//	uint8_t pesan[50];
	Vx = VX;
	Vy = VY;
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

//	// hitng sudut roda relatif pada body robot
//	float sudut_M1 = -(atan2(V1y, V1x))*180/pi+30;
//	float sudut_M2 = -(atan2(V2y, V2x))*180/pi-120+30;
//	float sudut_M3 = -(atan2(V3y, V3x))*180/pi+120+30;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);
	//HAL_Delay(5);
	X.speed = speed_M2;

	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);
	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
}

void Kinematik(float Vx, float Vy, float omega_deg) {
	PROF_BEGIN(PROF_SWERVE_KINEMATIK);
	roda X;
//	uint8_t pesan[50];
	Vx *= -1;
	Vy *= -1;
	omega_deg *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	LastSpeed1 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);

	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	LastSpeed2 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);

	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	LastSpeed3 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);

	PROF_END(PROF_SWERVE_KINEMATIK);
}

void RodaStop(void) {
	PROF_BEGIN(PROF_SWERVE_RODA_STOP);
//	uint8_t pesan[20];
//	sprintf(pesan, "stop di sini\n");
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
//	sprintf(pesan,"data=a1:%.1f a2:%.1f a3:%.1f \n", last_sudut_M1, last_sudut_M1, last_sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
//	osDelay();
	SetKecepatanDanSudutRoda(0, last_sudut_M1 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, last_sudut_M3 + Offset3, IdRoda3);
	SetKecepatanDanSudutRoda(0, last_sudut_M2 + Offset2, IdRoda2);
	PROF_END(PROF_SWERVE_RODA_STOP);
}

void RodaStopRemDepan(void) {
	SetKecepatanDanSudutRoda(0, 90 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, last_sudut_M3 + Offset3, IdRoda3);
	SetKecepatanDanSudutRoda(0, last_sudut_M2 + Offset2, IdRoda2);
}
//void RodaStopRemDepan(void)
//{
//	SetKecepatanDanSudutRoda(0,90+Offset1,IdRoda1);
//	SetKecepatanDanSudutRoda(0,last_sudut_M3+Offset3,IdRoda3);
//	SetKecepatanDanSudutRoda(0,last_sudut_M2+Offset2,IdRoda2);
//}

void RodaStopRemSemua(void) {
	SetKecepatanDanSudutRoda(0, 90 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, 45 + Offset3, IdRoda3);
	SetKecepatanDanSudutRoda(0, last_sudut_M2 + Offset2, IdRoda2);
}

void Parkir() {
	SetKecepatanDanSudutRoda(0, 90 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, 45 + Offset2, IdRoda2);
	SetKecepatanDanSudutRoda(0, -45 + Offset3, IdRoda3);
}

int count_window = 0;

float DegToRad = PI / 180;

void Kinematikhh(float Vx, float Vy, float omega_deg) {
	roda X;
	Vx *= 1;
	Vy *= 1;
	float Vw = omega_deg;
	float Rrobot = 1;

	float omega = omega_deg / 180.0f * PI;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (sin((90 + 45) * DegToRad) * Vw * Rrobot) + Vx;
	float V1y = -(cos((90 + 45) * DegToRad) * Vw * Rrobot) + Vy;

	float V2x = -(sin((270 + 45) * DegToRad) * Vw * Rrobot) + Vx;
	float V2y = -(cos((270 + 45) * DegToRad) * Vw * Rrobot) + Vy;

	float V3x = -(sin((90 + 45) * DegToRad) * Vw * Rrobot) + Vx;
	float V3y = (cos((90 + 45) * DegToRad) * Vw * Rrobot) + Vy;

	float V4x = (sin((270 + 45) * DegToRad) * Vw * Rrobot) + Vx;
	float V4y = (cos((270 + 45) * DegToRad) * Vw * Rrobot) + Vy;

	int16_t speed_M1 = (int16_t) (sqrt((V1x * V1x) + (V1y * V1y)));
	int16_t speed_M2 = (int16_t) -(sqrt((V2x * V2x) + (V2y * V2y)));
	int16_t speed_M3 = (int16_t) (sqrt((V3x * V3x) + (V3y * V3y)));
	int16_t speed_M4 = (int16_t) -(sqrt((V4x * V4x) + (V4y * V4y)));

	float SudutM1 = (atan2(V1y, V1x)) * 180 / PI;
	float SudutM2 = (atan2(V2y, V2x)) * 180 / PI;
	float SudutM3 = (atan2(V3y, V3x)) * 180 / PI;
	float SudutM4 = (atan2(V4y, V4x)) * 180 / PI;

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;
	if (speed_M4 > 127)
		speed_M4 = 127;
	else if (speed_M4 < -127)
		speed_M4 = -127;

	// hitng batas maksimal sudut
	if (SudutM1 > 180)
		SudutM1 -= 360;
	else if (SudutM1 < -180)
		SudutM1 += 360;

	if (SudutM2 > 180)
		SudutM2 -= 360;
	else if (SudutM2 < -180)
		SudutM2 += 360;

	if (SudutM3 > 180)
		SudutM3 -= 360;
	else if (SudutM3 < -180)
		SudutM3 += 360;

	if (SudutM4 > 180)
		SudutM4 -= 360;
	else if (SudutM4 < -180)
		SudutM4 += 360;

//	SetKecepatanDanSudutRoda( speed_M1,SudutM1+Offset1,IdRoda1);
//	SetKecepatanDanSudutRoda( speed_M2,SudutM2+Offset2,IdRoda2);
//	SetKecepatanDanSudutRoda( speed_M3,SudutM3+Offset3, IdRoda3);
//	SetKecepatanDanSudutRoda( speed_M4,SudutM4+Offset4, IdRoda4);

	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, SudutM1, &X);
	last_sudut_M1 = X.sudut;
	LastSpeed1 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);

	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, SudutM2, &X);
	last_sudut_M2 = X.sudut;
	LastSpeed2 = X.speed;

	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);

	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, SudutM3, &X);
	last_sudut_M3 = X.sudut;
	LastSpeed3 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);

	//HAL_Delay(5);
	X.speed = speed_M4;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M4, SudutM4, &X);
	last_sudut_M4 = X.sudut;
	LastSpeed3 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset4, IdRoda4);
//
//
//  moving_window_break[0][count_window] = last_sudut_M2;
//  moving_window_break[1][count_window] = last_sudut_M3;
//  if(count_window == 4) count_window = 0;
//  count_window ++;
}

void KinematikMajuAbs(float Vx, float Vy, float omega_deg) {

	roda X;
//	uint8_t pesan[50];
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

	SetKecepatanDanSudutRoda(speed_M1, sudut_M1 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(speed_M2, sudut_M2 + Offset3, IdRoda3);
	SetKecepatanDanSudutRoda(speed_M3, sudut_M3 + Offset2, IdRoda2);
}

void KinematikDepan(float Vx, float Vy, float omega_deg) {
	roda X;
//	uint8_t pesan[50];
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	LastSpeed1 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + OffsetDepanKiri2, IdRoda2);

	//HAL_Delay(5);
	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	LastSpeed2 = X.speed;

	SetKecepatanDanSudutRoda(X.speed, X.sudut + OffsetDepanKiri1, IdRoda1);

	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	LastSpeed3 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + OffsetDepanKiri3, IdRoda3);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

//	VxJoyTampil = X.sudut;
}

void KinematikReverse(float Vx, float Vy, float omega_deg) {
	roda X;
//	uint8_t pesan[50];
//	Vx*=-1;
//	Vy*=-1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	LastSpeed1 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);

	//HAL_Delay(5);
	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	LastSpeed2 = X.speed;

	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);

	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	LastSpeed3 = X.speed;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

//	VxJoyTampil = X.sudut;
}

void KinematikSpeedOffParkir(float Vx, float Vy, float omega_deg) {
	roda X;
//	uint8_t pesan[50];
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + 90 + Offset1, IdRoda1);
	//HAL_Delay(5);
	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + Offset3, IdRoda3);
	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

//	VxJoyTampil = X.sudut;
}

void KinematikSpeedOff(float Vx, float Vy, float omega_deg) {
	roda X;
//	uint8_t pesan[50];
	Vx *= -1;
	Vy *= -1;
	float omega = omega_deg / 180.0f * pi;
	float VR = sqrt((Vx * Vx) + (Vy * Vy));

	float V1x = (omega * r) + Vx;
	float V1y = Vy;

	float V2x = (-omega * r * Sin30) + Vx;
	float V2y = (-omega * r * Sin60) + Vy;

	float V3x = (-omega * r * Sin30) + Vx;
	float V3y = (omega * r * Sin60) + Vy;

	int16_t speed_M1 = (int16_t) sqrt((V1x * V1x) + (V1y * V1y));
	int16_t speed_M2 = (int16_t) sqrt((V2x * V2x) + (V2y * V2y));
	int16_t speed_M3 = (int16_t) sqrt((V3x * V3x) + (V3y * V3y));

	// set batas kecepatan maksimal motor
	if (speed_M1 > 127)
		speed_M1 = 127;
	else if (speed_M1 < -127)
		speed_M1 = -127;
	if (speed_M2 > 127)
		speed_M2 = 127;
	else if (speed_M2 < -127)
		speed_M2 = -127;
	if (speed_M3 > 127)
		speed_M3 = 127;
	else if (speed_M3 < -127)
		speed_M3 = -127;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = (atan2(V1y, V1x)) * 180 / pi;
	float sudut_M2 = (atan2(V2y, V2x)) * 180 / pi - 0;
	float sudut_M3 = (atan2(V3y, V3x)) * 180 / pi + 0;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = speed_M1;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + Offset1, IdRoda1);
	//HAL_Delay(5);
	X.speed = speed_M2;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + Offset3, IdRoda3);
	//HAL_Delay(5);
	X.speed = speed_M3;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	SetKecepatanDanSudutRoda(0, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

//	VxJoyTampil = X.sudut;
}

void IncrementSudutSwerve(float Sudut) {
	roda X;

	// hitng sudut roda relatif pada body robot
	float sudut_M1 = -(Sudut);
	float sudut_M2 = -(Sudut) - 120;
	float sudut_M3 = -(Sudut) + 120;

	// hitng batas maksimal sudut
	if (sudut_M1 > 180)
		sudut_M1 -= 360;
	else if (sudut_M1 < -180)
		sudut_M1 += 360;

	if (sudut_M2 > 180)
		sudut_M2 -= 360;
	else if (sudut_M2 < -180)
		sudut_M2 += 360;

	if (sudut_M3 > 180)
		sudut_M3 -= 360;
	else if (sudut_M3 < -180)
		sudut_M3 += 360;

//	sprintf( pesan,"%.1f,\n",sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);

	// atur arah putaran serta sdt roda berdasarkan perbahan gerak yang baru
	X.speed = 0;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M1, sudut_M1, &X);
	last_sudut_M1 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset1, IdRoda1);
	//HAL_Delay(5);
	X.speed = 0;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M2, sudut_M2, &X);
	last_sudut_M2 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset3, IdRoda3);
	//HAL_Delay(5);
	X.speed = 0;
	if (flag_puter_roda)
		X.speed = 0; //memutar roda di tempat
	hitungSudutSpeed(last_sudut_M3, sudut_M3, &X);
	last_sudut_M3 = X.sudut;
	SetKecepatanDanSudutRoda(X.speed, X.sudut + Offset2, IdRoda2);
	//HAL_Delay(5);
//	sprintf(pesan,"data=v1:%d v2:%d v3:%d a1:%.1f a2:%.1f a3:%.1f \n", speed_M1, speed_M2, speed_M3, sudut_M1, sudut_M2, sudut_M3);
//  HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
}

void RodaNol_Abs(void) {

	HAL_Delay(2);
	SetKecepatanDanSudutRoda(0, -90 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, -90 + Offset2, IdRoda2);
	SetKecepatanDanSudutRoda(0, -90 + Offset3, IdRoda3);

}

void RodaStopRemKananKiri(void) {
//	uint8_t pesan[20];
//	sprintf(pesan, "stop di sini\n");
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
//	sprintf(pesan,"data=a1:%.1f a2:%.1f a3:%.1f \n", last_sudut_M1, last_sudut_M1, last_sudut_M1);
//	HAL_UART_Transmit(&huart3, pesan, strlen(pesan), 1000);
	HAL_Delay(2);
	SetKecepatanDanSudutRoda(0, -20 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, 20 + Offset2, IdRoda2);
	SetKecepatanDanSudutRoda(0, 20 + Offset3, IdRoda3);

}

int average_roda2, average_roda3;

void RodaStopRemDepanAbs(void) {
	SetKecepatanDanSudutRoda(0, -90 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(0, -135 + Offset2, IdRoda2);
	SetKecepatanDanSudutRoda(0, -45 + Offset3, IdRoda3);
}

float StopSpeed1 = 0, StopSpeed2 = 0, StopSpeed3 = 0;
void StopSpeed(int Time) {

	//Speed 1
	if (LastSpeed1 >= 0)
		StopSpeed1 = -LastSpeed1 * 1.25;
	else
		StopSpeed1 = -LastSpeed1 * 1.25;
	//Speed 2
	if (LastSpeed2 >= 0)
		StopSpeed2 = -LastSpeed2 * 1.25;
	else
		StopSpeed2 = -LastSpeed2 * 1.25;
	//Speed 3
	if (LastSpeed3 >= 0)
		StopSpeed3 = -LastSpeed3 * 1.25;
	else
		StopSpeed3 = -LastSpeed3 * 1.25;

	//Batas Speed1
	if (StopSpeed1 > 127)
		StopSpeed1 = 127;
	else if (StopSpeed1 < -127)
		StopSpeed1 = -127;

	//Batas Speed2
	if (StopSpeed2 > 127)
		StopSpeed2 = 127;
	else if (StopSpeed2 < -127)
		StopSpeed2 = -127;

	//Batas Speed3
	if (StopSpeed3 > 127)
		StopSpeed3 = 127;
	else if (StopSpeed3 < -127)
		StopSpeed3 = -127;

	SetKecepatanDanSudutRoda(StopSpeed1, last_sudut_M1 + Offset1, IdRoda1);
	SetKecepatanDanSudutRoda(StopSpeed3, last_sudut_M3 + Offset2, IdRoda2);
	SetKecepatanDanSudutRoda(StopSpeed2, last_sudut_M2 + Offset3, IdRoda3);
	HAL_Delay(Time);

	RodaStop();
}

/*
 * ============================================================
 * RIZAL Dynamic Braking
 * ============================================================
 * Fungsi ini tidak mengganti RodaStop().
 * RodaStop() tetap aman untuk stop normal / emergency.
 *
 * Dynamic brake bekerja dengan cara mengirim command balik arah
 * dari command gerak terakhir yang dikirim oleh JalanKeTitikRIZAL()
 * atau JalanKeTitikFinalHeadingRIZAL().
 *
 * Cara pakai awal setelah waypoint otomatis:
 *   RodaBrakeDynamicRIZAL(0.35f, 30.0f, 35U);
 *
 * Tuning:
 *   gain      : 0.20f - 0.50f
 *   max_brake : 20.0f - 45.0f
 *   brake_ms  : 20U   - 60U
 */
#define R1_BRAKE_MIN_TRANSLATION_CMD   1.0f
#define R1_BRAKE_MIN_OMEGA_CMD         0.5f
#define R1_BRAKE_OS_DELAY_MS           5U

float R1_BRAKE_DBG_LastVx = 0.0f;
float R1_BRAKE_DBG_LastVy = 0.0f;
float R1_BRAKE_DBG_LastOmega = 0.0f;
float R1_BRAKE_DBG_CmdVx = 0.0f;
float R1_BRAKE_DBG_CmdVy = 0.0f;
float R1_BRAKE_DBG_CmdOmega = 0.0f;
uint32_t R1_BRAKE_DBG_LastMs = 0U;

static void RodaBrake_UpdateLastCommandRIZAL(float vx, float vy, float omega)
{
	float mag;

	mag = sqrtf((vx * vx) + (vy * vy));

	if ((mag > R1_BRAKE_MIN_TRANSLATION_CMD) ||
		(absf(omega) > R1_BRAKE_MIN_OMEGA_CMD)) {
		R1_BRAKE_DBG_LastVx = vx;
		R1_BRAKE_DBG_LastVy = vy;
		R1_BRAKE_DBG_LastOmega = omega;
	}
}

void RodaBrakeResetLastCommandRIZAL(void)
{
	R1_BRAKE_DBG_LastVx = 0.0f;
	R1_BRAKE_DBG_LastVy = 0.0f;
	R1_BRAKE_DBG_LastOmega = 0.0f;
	R1_BRAKE_DBG_CmdVx = 0.0f;
	R1_BRAKE_DBG_CmdVy = 0.0f;
	R1_BRAKE_DBG_CmdOmega = 0.0f;
	R1_BRAKE_DBG_LastMs = 0U;
}

void RodaBrakeDynamicRIZAL(float gain, float max_brake, uint32_t brake_ms)
{
	float bx;
	float by;
	float bo;
	float mag;
	uint32_t elapsed;

	if (gain < 0.0f) {
		gain = -gain;
	}

	if (max_brake < 0.0f) {
		max_brake = -max_brake;
	}

	if ((gain <= 0.0f) || (max_brake <= 0.0f) || (brake_ms == 0U)) {
		RodaStop();
		return;
	}

	/* Command balik arah dari command gerak terakhir. */
	bx = -R1_BRAKE_DBG_LastVx * gain;
	by = -R1_BRAKE_DBG_LastVy * gain;
	bo = -R1_BRAKE_DBG_LastOmega * gain;

	mag = sqrtf((bx * bx) + (by * by));

	if (mag > max_brake && mag > 0.001f) {
		bx = bx * max_brake / mag;
		by = by * max_brake / mag;
	}

	if (bo > max_brake) {
		bo = max_brake;
	} else if (bo < -max_brake) {
		bo = -max_brake;
	}

	R1_BRAKE_DBG_CmdVx = bx;
	R1_BRAKE_DBG_CmdVy = by;
	R1_BRAKE_DBG_CmdOmega = bo;
	R1_BRAKE_DBG_LastMs = brake_ms;

	if ((mag < R1_BRAKE_MIN_TRANSLATION_CMD) &&
		(absf(bo) < R1_BRAKE_MIN_OMEGA_CMD)) {
		RodaStop();
		return;
	}

	elapsed = 0U;
	while (elapsed < brake_ms) {
		if (FlagPaksa || PS_INT) {
			break;
		}

		Kinematik(bx, by, bo);

		osDelay(R1_BRAKE_OS_DELAY_MS);
		elapsed += R1_BRAKE_OS_DELAY_MS;
	}

	RodaStop();
}

unsigned short BacaEnkoderA(void)	// menggunakan konektor encoder 1
{
	return __HAL_TIM_GET_COUNTER(&htim3);
}
unsigned short BacaEnkoderB(void)	// menggunakan konektor encoder 2
{
	return __HAL_TIM_GET_COUNTER(&htim1);
}

void SetWorldPosition_X_and_Y(float x, float y) {
	PosX_World = x;
	PosY_World = y;
	;
}

void SetHeading(float h) {
	YawOffset = (-h + DataYawIMU);
}
void SetWorldPosition(float x, float y, float h) {
	PosX_World = x;
	PosY_World = y;
	YawOffset = (-h + DataYawIMU);
}

void SetWorldPosHeading0(float x, float y) {
	PosX_World = x;
	PosY_World = y;
}
// memaksa nilai posisi dan orientasi robot ke nilai origin baru
void ResetWorldPosition(void) {
	PosX_World = 0;
	PosY_World = 0;
	YawOffset = DataYawIMU;
}

void speed_Encoder(void) {
	SpeedEncA = BacaEnkoderA() - 32767;
	SpeedEncB = BacaEnkoderB() - 32767;
	//reset
	TIM3->CNT = 32767;
	TIM1->CNT = 32767;
}

char Pesan[30];
long T = 0;

void BacaPosisiEncoder(void) {

//  SpeedEnc = 32768-BacaEnkoderB();
//  TIM2->CNT = 32768;
//  Rpm = (SpeedEnc*6000)/-8271;

	//uint8_t pesan[30];
	static uint16_t lastEncA = 0, lastEncB = 0;	//, lastEncC=0;	// untuk menyimpan data encoder sebelumnya
	uint16_t EncoderBuffer;				// buffer sementara pembacaan encoder
	long a;				// variabel pembantu untuk perhitungan lebih dari 16 bit
	float PosX_Robot, PosY_Robot, Yaw;//perubahan posisi dan orientasi (heading robot)
	// hitung perubahan data EncoderA	---> Kanan  --> arah putaran terbalik dari asumsi (data dinegatifkan)
	EncoderBuffer = BacaEnkoderA();
	a = (long) EncoderBuffer - (long) lastEncA;
	if (a < -32000)												// naik overflow
		SpeedEncA = -(short) (a + 65536);		// nilai +
	else if (a > 32000)										// turun overflow
		SpeedEncA = -(short) (a - 65536);		// nilai -
	else
		// normal
		SpeedEncA = -(short) a;							// nilai +=naik, -=turun
	lastEncA = EncoderBuffer;// simpan data sekarang untuk sampling berikutnya
	// hitung perubahan data EncoderB	---> Kiri --> arah putaran sesuai, sehingga polaritas dibuat tetap
	EncoderBuffer = BacaEnkoderB();
	a = (long) EncoderBuffer - (long) lastEncB;
	if (a < -32000)	// naik overflow
		SpeedEncB = (short) (a + 65536);
	else if (a > 32000)	// turun overflow
		SpeedEncB = (short) (a - 65536);
	else
		SpeedEncB = (short) a;
	lastEncB = EncoderBuffer;

//		PosX_Robot = EncoderConstant*(float)(-2*SpeedEncC+SpeedEncB+SpeedEncA);//*2660/4175;
//		PosY_Robot = EncoderConstant*(float)(SpeedEncB-SpeedEncA);//*AkarTiga*2660/4125;
	PosX_Robot = EncoderConstant
			* (float) ((SpeedEncA - SpeedEncB) * Sin60 * 0.5F) * 4000.0f
			/ 900.0f;
	PosY_Robot = EncoderConstant
			* (float) ((SpeedEncA + SpeedEncB) * Sin30 * 0.5F) * 4000.0f
			/ 900.0f;
	//PosH_Robot = EncoderConstant*(float)(SpeedEncC+SpeedEncB+SpeedEncA)/(0.6*330/360*RadiusRobot);//

	//PosX_Robot = -PosX_Robot;
	Yaw = DataYawIMU - YawOffset;
	//Yaw *=-1;

	if (Yaw > 180)
		Yaw -= 360;		//membatasi agar -180 sd +180
	else if (Yaw < -180)
		Yaw += 360;

//		if(Yaw<=0) Yaw+=180;
//		else if(Yaw>=0) Yaw-=180;

	RobotYaw = Yaw;
	//RobotYaw=0;
//		sprintf(pesan,"%d %d\n", (int)PosX_Robot, (int)PosX_Robot);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
//		sprintf(Usart3DmaTxBuffer, "x%d\n",(int)Yaw);
//		KirimUsart3_DMA();
//
	//PosH_World += PosH_Robot*Rad2Deg;		// sudut heading robot terhadap lapangan dalam derajat
//		PosY_World += PosX_Robot*cos(RobotYaw*Deg2Rad) + PosY_Robot*sin(RobotYaw*Deg2Rad);  // posisi x robot terhadap lapangan dalam mm
//		PosX_World -= PosX_Robot*sin(RobotYaw*Deg2Rad)  +PosY_Robot*cos(RobotYaw*Deg2Rad);	// posisi y robot terhadap lapangan dalam mm
//		sprintf(pesan,"%d %d\n", (int)PosX_World, (int)PosY_World);
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));
//		sprintf(pesan,"%d %d %d.%d\n", (int)PosX_World, (int)PosY_World, (int)RobotYaw, (int) absf(RobotYaw*10));
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));

	//RIZAL
	/*
	 * ODOMETRY LEGACY - sesuai arah koordinat program lama
	 *
	 * Koordinat yang dipakai JalanKeTitik:
	 * X+ = kanan lapangan
	 * Y+ = maju lapangan
	 *
	 * Catatan:
	 * Nama PosX_Robot / PosY_Robot di sini jangan dianggap
	 * sebagai X kanan dan Y depan secara matematis standar,
	 * karena hasil encoder robot ini sudah punya orientasi sendiri.
	 */
	PosY_World += PosX_Robot * cosf(RobotYaw * Deg2Rad)
			+ PosY_Robot * sinf(RobotYaw * Deg2Rad);

	PosX_World -= PosX_Robot * sinf(RobotYaw * Deg2Rad)
			+ PosY_Robot * cosf(RobotYaw * Deg2Rad);
	togglePosisi++;

	R1_DBG_PosX_Robot = PosX_Robot;
	R1_DBG_PosY_Robot = PosY_Robot;
	R1_DBG_PosX_World = PosX_World;
	R1_DBG_PosY_World = PosY_World;
	R1_DBG_RobotYaw = RobotYaw;
	R1_DBG_SpeedEncA = SpeedEncA;
	R1_DBG_SpeedEncB = SpeedEncB;

}

void JalanKeTitik(float TargetX, float TargetY, float TargetH, float BatasError,
		char sFlag, int StartSpeed) {
	float u, ErrorX, ErrorY, ErrorH, ErrorJ, dErrorJ, dErrorH, lastErrorJ,
			lastErrorH;
	char pesan[50];
	float sudut;
	float Vx, Vy, SyaratStop;
	static unsigned char lastTogglePosisi;
	static float CurrentSpeed = 1500;
//		static int counter=0;
	static long Kirim = 0;

	lastTogglePosisi = togglePosisi + 1;
	ErrorX = TargetX - PosX_World;
	ErrorY = TargetY - PosY_World;
	lastErrorH = TargetH - RobotYaw;
	lastErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
	if (StartSpeed != 0)
		CurrentSpeed = StartSpeed;
	do {
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}
		//
		// BacaPosisiEncoder();		// menbaca posisi koordinat robot terhadap lapangan
		// menghitung PID Jarak
//		while(togglePosisi==lastTogglePosisi);	// menunggu sampling data baru
		lastTogglePosisi = togglePosisi;

		if (CurrentSpeed >= MaxSpeed)
			CurrentSpeed = MaxSpeed;
		else
			CurrentSpeed += 1000;

		ErrorX = TargetX - PosX_World;
		ErrorY = TargetY - PosY_World;
		ErrorH = TargetH - RobotYaw;

		ErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
		sudut = atan2(ErrorX, ErrorY) * Rad2Deg;

		sudut -= RobotYaw;// menghitung nilai relatif terhadap orientasi global
		if (sudut > 180)
			sudut -= 360;		//membatasi agar -180 sd +180
		else if (sudut < -180)
			sudut += 360;
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;	//membatasi agar -180 sd +180
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;

		//membatasi nilai Error Jarak
		//if(absf(ErrorJ)<1000)SpeedStatus=2;
		SetBatas(&ErrorJ, LimitErrorJ);	// error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorJ = ErrorJ - lastErrorJ;
		SetBatas(&dErrorJ, LimitErrorJ);// delta error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorJ = ErrorJ;

		//iErrorJ+=(ErrorJ*SkalaErrorJ);
		//SetBatas(&iErrorJ, 1000);
		u = KPj * ErrorJ + KDj * dErrorJ;// + KIj * iErrorJ;	// menghitung control output untuk jarak

		SetBatas(&u, CurrentSpeed);				// batas kecepatan
		u *= 0.0254f;				// merubah skala kecepatan dari 5000 ke 127

		Vx = u * sin(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu x
		Vy = u * cos(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu y

		SetBatas(&ErrorH, LimitErrorH);	// error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorH = ErrorH - lastErrorH;
		SetBatas(&dErrorH, LimitErrorH);// delta error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorH = ErrorH;
		u = KPh * ErrorH + KDh * dErrorH;// + KIh * iErrorH;	// menghitung control output untuk Heading
		u *= 1;						// merubah skala kecepatan dari 5000 ke 127

		Kinematik(Vx, Vy, u);							//targetH = RobotYaw;

		//targetH = RobotYaw;

		if (sFlag == 0)
			SyaratStop = absf(ErrorJ) - BatasError;
		else if (sFlag == 1)
			SyaratStop = absf(ErrorH) - BatasError;

		if (flag_puter_roda)
			SyaratStop = 0;
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}
		//StatusKD=1;
		osDelay(5); // rizal
	} while (SyaratStop > 0); // || absf(ErrorH)>BatasErrorHeading);	// menunggu sampai posisi dan arah arah hadap dibawah toleransi

}

/*
 * ============================================================
 * JalanKeTitikRIZAL
 * ============================================================
 * Versi aman untuk sequence R1.
 * Original JalanKeTitik() tidak diubah.
 *
 * Perbedaan utama:
 * - Error jarak asli dan error heading asli disimpan sebagai RAW.
 * - Error yang dibatasi LimitErrorJ / LimitErrorH hanya dipakai untuk PID.
 * - Syarat stop memakai RAW error, sehingga mode heading tidak selesai palsu
 *   hanya karena ErrorH sudah dibatasi oleh LimitErrorH.
 *
 * sFlag:
 * - 0 = stop berdasarkan jarak RAW.
 * - 1 = stop berdasarkan heading RAW.
 */
void JalanKeTitikRIZAL(float TargetX, float TargetY, float TargetH,
		float BatasError, char sFlag, int StartSpeed) {
	float u;
	float ErrorX;
	float ErrorY;
	float ErrorH_Raw;
	float ErrorJ_Raw;
	float ErrorH_PID;
	float ErrorJ_PID;
	float dErrorJ;
	float dErrorH;
	float lastErrorJ;
	float lastErrorH;
	float sudut;
	float Vx;
	float Vy;
	float SyaratStop;
	static unsigned char lastTogglePosisi;
	static float CurrentSpeed = 1500;

	lastTogglePosisi = togglePosisi + 1;

	ErrorX = TargetX - PosX_World;
	ErrorY = TargetY - PosY_World;

	ErrorJ_Raw = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
	ErrorH_Raw = TargetH - RobotYaw;

	if (ErrorH_Raw > 180.0f) {
		ErrorH_Raw -= 360.0f;
	} else if (ErrorH_Raw < -180.0f) {
		ErrorH_Raw += 360.0f;
	}

	ErrorJ_PID = ErrorJ_Raw;
	SetBatas(&ErrorJ_PID, LimitErrorJ);
	lastErrorJ = ErrorJ_PID;

	ErrorH_PID = ErrorH_Raw;
	SetBatas(&ErrorH_PID, LimitErrorH);
	lastErrorH = ErrorH_PID;

	if (StartSpeed != 0) {
		CurrentSpeed = StartSpeed;
	}

	do {
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}

		lastTogglePosisi = togglePosisi;

		if (CurrentSpeed >= MaxSpeed) {
			CurrentSpeed = MaxSpeed;
		} else {
			CurrentSpeed += 1000;
		}

		ErrorX = TargetX - PosX_World;
		ErrorY = TargetY - PosY_World;

		ErrorJ_Raw = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
		ErrorH_Raw = TargetH - RobotYaw;

		if (ErrorH_Raw > 180.0f) {
			ErrorH_Raw -= 360.0f;
		} else if (ErrorH_Raw < -180.0f) {
			ErrorH_Raw += 360.0f;
		}

		sudut = atan2(ErrorX, ErrorY) * Rad2Deg;
		sudut -= RobotYaw;

		if (sudut > 180.0f) {
			sudut -= 360.0f;
		} else if (sudut < -180.0f) {
			sudut += 360.0f;
		}

		/* PID jarak memakai error yang dibatasi */
		ErrorJ_PID = ErrorJ_Raw;
		SetBatas(&ErrorJ_PID, LimitErrorJ);

		dErrorJ = ErrorJ_PID - lastErrorJ;
		SetBatas(&dErrorJ, LimitErrorJ);
		lastErrorJ = ErrorJ_PID;

		u = (KPj * ErrorJ_PID) + (KDj * dErrorJ);
		SetBatas(&u, CurrentSpeed);
		u *= 0.0254f;

		Vx = u * sin(sudut * Deg2Rad);
		Vy = u * cos(sudut * Deg2Rad);

		/* PID heading memakai error yang dibatasi */
		ErrorH_PID = ErrorH_Raw;
		SetBatas(&ErrorH_PID, LimitErrorH);

		dErrorH = ErrorH_PID - lastErrorH;
		SetBatas(&dErrorH, LimitErrorH);
		lastErrorH = ErrorH_PID;

		u = (KPh * ErrorH_PID) + (KDh * dErrorH);

		RodaBrake_UpdateLastCommandRIZAL(Vx, Vy, u);
		Kinematik(Vx, Vy, u);

		/* Syarat stop memakai RAW error, bukan error PID yang sudah dibatasi */
		if (sFlag == 0) {
			SyaratStop = absf(ErrorJ_Raw) - BatasError;
		} else if (sFlag == 1) {
			SyaratStop = absf(ErrorH_Raw) - BatasError;
		} else {
			SyaratStop = 0;
		}

		if (flag_puter_roda) {
			SyaratStop = 0;
		}

		if (PS_INT) {
			SyaratStop = 0;
			RodaStop();
			break;
		}

		R1_DBG_ErrorX = ErrorX;
		R1_DBG_ErrorY = ErrorY;
		R1_DBG_ErrorJ = ErrorJ_Raw;
		R1_DBG_ErrorH = ErrorH_Raw;
		R1_DBG_Sudut = sudut;
		R1_DBG_Vx = Vx;
		R1_DBG_Vy = Vy;
		R1_DBG_Omega = u;

		osDelay(5);
	} while (SyaratStop > 0);
}

void JalanKeTitikFinalHeadingRIZAL(float TargetX, float TargetY, float TargetH,
		float BatasJarak, float BatasHeading, int StartSpeed) {
	float u;
	float ErrorX;
	float ErrorY;
	float ErrorJ_Raw;
	float ErrorH_Raw;
	float ErrorJ_PID;
	float ErrorH_PID;
	float dErrorJ;
	float dErrorH;
	float lastErrorJ = 0.0f;
	float lastErrorH = 0.0f;
	float sudut;
	float Vx;
	float Vy;
	float omega;
	static float CurrentSpeed = 1500.0f;

	if (StartSpeed != 0) {
		CurrentSpeed = StartSpeed;
	}

	ErrorX = TargetX - PosX_World;
	ErrorY = TargetY - PosY_World;
	ErrorJ_Raw = sqrtf((ErrorX * ErrorX) + (ErrorY * ErrorY));

	ErrorH_Raw = TargetH - RobotYaw;
	if (ErrorH_Raw > 180.0f)
		ErrorH_Raw -= 360.0f;
	else if (ErrorH_Raw < -180.0f)
		ErrorH_Raw += 360.0f;

	lastErrorJ = ErrorJ_Raw;
	lastErrorH = ErrorH_Raw;

	do {
		if (FlagPaksa) {
			break;
		}

		if (CurrentSpeed < MaxSpeed) {
			CurrentSpeed += 1000.0f;
		} else {
			CurrentSpeed = MaxSpeed;
		}

		ErrorX = TargetX - PosX_World;
		ErrorY = TargetY - PosY_World;
		ErrorJ_Raw = sqrtf((ErrorX * ErrorX) + (ErrorY * ErrorY));

		ErrorH_Raw = TargetH - RobotYaw;
		if (ErrorH_Raw > 180.0f)
			ErrorH_Raw -= 360.0f;
		else if (ErrorH_Raw < -180.0f)
			ErrorH_Raw += 360.0f;

		/*
		 * Jika posisi belum sampai, translasi masih aktif.
		 * Jika posisi sudah masuk batas, translasi dimatikan
		 * dan robot hanya menyelesaikan heading.
		 */
		if (ErrorJ_Raw > BatasJarak) {
			sudut = atan2f(ErrorX, ErrorY) * Rad2Deg;
			sudut -= RobotYaw;

			if (sudut > 180.0f)
				sudut -= 360.0f;
			else if (sudut < -180.0f)
				sudut += 360.0f;

			ErrorJ_PID = ErrorJ_Raw;
			SetBatas(&ErrorJ_PID, LimitErrorJ);

			dErrorJ = ErrorJ_PID - lastErrorJ;
			SetBatas(&dErrorJ, LimitErrorJ);
			lastErrorJ = ErrorJ_PID;

			u = (KPj * ErrorJ_PID) + (KDj * dErrorJ);
			SetBatas(&u, CurrentSpeed);
			u *= 0.0254f;

			Vx = u * sinf(sudut * Deg2Rad);
			Vy = u * cosf(sudut * Deg2Rad);
		} else {
			/*
			 * Sudah dekat target posisi.
			 * Jangan jalan kanan/kiri lagi.
			 */
			Vx = 0.0f;
			Vy = 0.0f;
		}

		ErrorH_PID = ErrorH_Raw;
		SetBatas(&ErrorH_PID, LimitErrorH);

		dErrorH = ErrorH_PID - lastErrorH;
		SetBatas(&dErrorH, LimitErrorH);
		lastErrorH = ErrorH_PID;

		omega = (KPh * ErrorH_PID) + (KDh * dErrorH);

		RodaBrake_UpdateLastCommandRIZAL(Vx, Vy, omega);
		Kinematik(Vx, Vy, omega);

		if (PS_INT != 0) {
			RodaStop();
			break;
		}

		osDelay(5);

		R1_DBG_ErrorX = ErrorX;
		R1_DBG_ErrorY = ErrorY;
		R1_DBG_ErrorJ = ErrorJ_Raw;
		R1_DBG_ErrorH = ErrorH_Raw;
		R1_DBG_Sudut = sudut;
		R1_DBG_Vx = Vx;
		R1_DBG_Vy = Vy;
		R1_DBG_Omega = omega;
	} while ((ErrorJ_Raw > BatasJarak) || (absf(ErrorH_Raw) > BatasHeading)
			|| PS_INT != 0);

	RodaStop();
}

void JalanKeTitikEdit(float TargetX, float TargetY, float TargetH,
		float BatasError, char sFlag, int StartSpeed) {
	float u, ErrorX, ErrorY, ErrorH, ErrorJ, dErrorJ, dErrorH, lastErrorJ,
			lastErrorH;
	char pesan[50];
	float sudut;
	float Vx, Vy, SyaratStop;
	static unsigned char lastTogglePosisi;
	static float CurrentSpeed = 1500;
//		static int counter=0;
	static long Kirim = 0;

	lastTogglePosisi = togglePosisi + 1;
	ErrorX = TargetX - PosX_World;
	ErrorY = TargetY - PosY_World;
	lastErrorH = TargetH - RobotYaw;
	lastErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
	if (StartSpeed != 0)
		CurrentSpeed = StartSpeed;
	do {
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}
		//
		BacaPosisiEncoder();// menbaca posisi koordinat robot terhadap lapangan
		// menghitung PID Jarak
		while (togglePosisi == lastTogglePosisi)
			;	// menunggu sampling data baru
		lastTogglePosisi = togglePosisi;

		if (CurrentSpeed >= MaxSpeed)
			CurrentSpeed = MaxSpeed;
		else
			CurrentSpeed += 1000;

		ErrorX = TargetX - PosX_World;
		ErrorY = TargetY - PosY_World;
		ErrorH = TargetH - RobotYaw;

		ErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
		sudut = atan2(ErrorX, ErrorY) * Rad2Deg;

		if (ErrorJ < 2500)
			MaxSpeed = 1500;
		sudut -= RobotYaw;// menghitung nilai relatif terhadap orientasi global
		if (sudut > 180)
			sudut -= 360;		//membatasi agar -180 sd +180
		else if (sudut < -180)
			sudut += 360;
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;	//membatasi agar -180 sd +180
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;

		//membatasi nilai Error Jarak
		//if(absf(ErrorJ)<1000)SpeedStatus=2;
		SetBatas(&ErrorJ, LimitErrorJ);	// error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorJ = ErrorJ - lastErrorJ;
		SetBatas(&dErrorJ, LimitErrorJ);// delta error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorJ = ErrorJ;

		//iErrorJ+=(ErrorJ*SkalaErrorJ);
		//SetBatas(&iErrorJ, 1000);
		u = KPj * ErrorJ + KDj * dErrorJ;// + KIj * iErrorJ;	// menghitung control output untuk jarak

		SetBatas(&u, CurrentSpeed);				// batas kecepatan
		u *= 0.0254f;				// merubah skala kecepatan dari 5000 ke 127

		Vx = u * sin(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu x
		Vy = u * cos(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu y

		SetBatas(&ErrorH, LimitErrorH);	// error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorH = ErrorH - lastErrorH;
		SetBatas(&dErrorH, LimitErrorH);// delta error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorH = ErrorH;
		u = KPh * ErrorH + KDh * dErrorH;// + KIh * iErrorH;	// menghitung control output untuk Heading
		u *= 1;						// merubah skala kecepatan dari 5000 ke 127

		Kinematik(Vx, Vy, u);							//targetH = RobotYaw;

		//targetH = RobotYaw;

		if (sFlag == 0)
			SyaratStop = absf(ErrorJ) - BatasError;
		else if (sFlag == 1)
			SyaratStop = absf(ErrorH) - BatasError;

		if (flag_puter_roda)
			SyaratStop = 0;
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}
		//StatusKD=1;
	} while (SyaratStop > 0);// || absf(ErrorH)>BatasErrorHeading);	// menunggu sampai posisi dan arah arah hadap dibawah toleransi

}

void JalanKeTitikHolonomic(float TargetX, float TargetY, float TargetH,
		float BatasError, char sFlag, int StartSpeed) {
	float u, ErrorX, ErrorY, ErrorH, ErrorJ, dErrorJ, dErrorH, lastErrorJ,
			lastErrorH;
	char pesan[50];
	float sudut;
	float Vx, Vy, SyaratStop;
	static unsigned char lastTogglePosisi;
	static float CurrentSpeed = 1500;
//		static int counter=0;
	static long Kirim = 0;

	lastTogglePosisi = togglePosisi + 1;
	ErrorX = TargetX - PosX_World;
	ErrorY = TargetY - PosY_World;
	lastErrorH = (sudut - RobotYaw) - TargetH;
	lastErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
	if (StartSpeed != 0)
		CurrentSpeed = StartSpeed;
	do {
		//BacaPosisiEncoder();		// menbaca posisi koordinat robot terhadap lapangan
		// menghitung PID Jarak
		while (togglePosisi == lastTogglePosisi)
			;	// menunggu sampling data baru
		lastTogglePosisi = togglePosisi;

		if (CurrentSpeed >= MaxSpeed)
			CurrentSpeed = MaxSpeed;
		else
			CurrentSpeed += 500;

		ErrorX = TargetX - PosX_World;
		ErrorY = TargetY - PosY_World;

		ErrorJ = sqrt(ErrorX * ErrorX + ErrorY * ErrorY);
		sudut = atan2(ErrorX, ErrorY) * Rad2Deg;

		ErrorH = (sudut - RobotYaw) - TargetH;

		sudut -= RobotYaw;// menghitung nilai relatif terhadap orientasi global
		if (sudut > 180)
			sudut -= 360;		//membatasi agar -180 sd +180
		else if (sudut < -180)
			sudut += 360;
		if (ErrorH > 180)
			ErrorH = ErrorH - 360;	//membatasi agar -180 sd +180
		else if (ErrorH < -180)
			ErrorH = ErrorH + 360;

		//membatasi nilai Error Jarak
		//if(absf(ErrorJ)<1000)SpeedStatus=2;
		SetBatas(&ErrorJ, LimitErrorJ);	// error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorJ = ErrorJ - lastErrorJ;
		SetBatas(&dErrorJ, LimitErrorJ);// delta error jarak dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorJ = ErrorJ;

		//iErrorJ+=(ErrorJ*SkalaErrorJ);
		//SetBatas(&iErrorJ, 1000);
		u = KPj * ErrorJ + KDj * dErrorJ;// + KIj * iErrorJ;	// menghitung control output untuk jarak

		SetBatas(&u, CurrentSpeed);				// batas kecepatan
		u *= 0.0254f;				// merubah skala kecepatan dari 5000 ke 127

		Vx = u * sin(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu x
		Vy = u * cos(sudut * Deg2Rad);	// menghitung vektor kecepatan sumbu y

		SetBatas(&ErrorH, LimitErrorH);	// error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		dErrorH = ErrorH - lastErrorH;
		SetBatas(&dErrorH, LimitErrorH);// delta error sudut dibatasi supaya saat target jauh tidak terjadi overshoot di awal
		lastErrorH = ErrorH;
		u = KPh * ErrorH + KDh * dErrorH;// + KIh * iErrorH;	// menghitung control output untuk Heading
		u *= 1;						// merubah skala kecepatan dari 5000 ke 127
//		sprintf(pesan, "%.1f, %.1f, %.1f\n", u, ErrorH, dErrorH);
//		sprintf(pesan,"%d %d %d.%d\n", (int)PosX_World, (int)PosY_World, (int)RobotYaw, (int) absf(RobotYaw*10));
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)pesan, strlen(pesan));

		//		//Monitoring GUI
//		if((Kirim%16)==0)
//		{
//			sprintf(pesan,"X%dY%dH%d", (int)PosX_World, (int)PosY_World, (int)(RobotYaw*10.0));
//			HAL_UART_Transmit(&huart3, (uint8_t*)pesan, strlen(pesan),100);
//		}

//		Kirim++;
//			if((Kirim%30)==0)
//			{
//				sprintf(pesan," Jalan Titik\n");
//				HAL_UART_Transmit(&huart3,(uint8_t*)pesan,sizeof(pesan),100);
//			}
//			Kirim++;
		Kinematik(Vx, Vy, u);							//targetH = RobotYaw;

		//targetH = RobotYaw;

		if (sFlag == 0)
			SyaratStop = absf(ErrorJ) - BatasError;
		else if (sFlag == 1)
			SyaratStop = absf(ErrorH) - BatasError;

		if (flag_puter_roda)
			SyaratStop = 0;
		if (FlagPaksa) {
			SyaratStop = 0;
			break;
		}
		//StatusKD=1;
	} while (SyaratStop > 0);// || absf(ErrorH)>BatasErrorHeading);	// menunggu sampai posisi dan arah arah hadap dibawah toleransi
}

void RotasiHeadingAja(float TargetH, float Batas) {
	float SyaratStop = 0;
	float ErrorH = 0;
	do {
		ErrorH = TargetH - RobotYaw;

		if (RobotYaw > TargetH) {
			Kinematik(0, 0, 13);
		} else if (RobotYaw < TargetH) {
			Kinematik(0, 0, -13);
		}
		SyaratStop = absf(ErrorH) - Batas;
	}

	while (SyaratStop > 0);
}

#define ODO_DEG2RAD 0.01745329252f

/*
 * Body coordinate:
 * X+ = kanan robot
 * Y+ = depan robot
 *
 * Posisi encoder dari titik tengah robot.
 */
#define ODO_R_POS_X_MM       296.796f
#define ODO_R_POS_Y_MM      -171.355f

#define ODO_L_POS_X_MM      -296.796f
#define ODO_L_POS_Y_MM      -171.355f

/*
 * Sudut arah rolling encoder.
 * Diukur dari depan robot.
 * Positif ke kanan robot.
 */
#define ODO_R_ANGLE_DEG     -60.0f
#define ODO_L_ANGLE_DEG      60.0f

/*
 * Ubah ke -1 kalau arah count encoder kebalik.
 */
#define ODO_R_DIR             1.0f
#define ODO_L_DIR             -1.0f

#define DiameterEncoder_RIZAL     47.0f
#define ResolusiEncoder_RIZAL     (2048.0f * 4.0f)
#define ODO_MM_PER_TICK     ((DiameterEncoder_RIZAL * PI) / ResolusiEncoder_RIZAL)

#define ODO_R_MM_PER_TICK     ODO_MM_PER_TICK
#define ODO_L_MM_PER_TICK     ODO_MM_PER_TICK

static int16_t ODO_LastEncR = 0;
static int16_t ODO_LastEncL = 0;

static float ODO_YawOffsetDeg = 0.0f;
static float ODO_LastYawDeg = 0.0f;

float ODO_DBG_sR;
float ODO_DBG_sL;
float ODO_DBG_Right;
float ODO_DBG_Forward;
float ODO_DBG_rotR;
float ODO_DBG_rotL;
float ODO_DBG_dYaw;
float ODO_DBG_YawNow;

static float NormalizeDeg180_RIZAL(float angle) {
	while (angle > 180.0f)
		angle -= 360.0f;
	while (angle < -180.0f)
		angle += 360.0f;
	return angle;
}

float Odo_GetRobotYawRIZAL(void) {
	/*
	 * DataYawIMU:
	 * - 0 sampai 360 derajat
	 * - CW bertambah
	 *
	 * RobotYaw:
	 * - relatif terhadap arena/program
	 * - -180 sampai 180 derajat
	 */
	return NormalizeDeg180_RIZAL(DataYawIMU - ODO_YawOffsetDeg);
}

void OdoDeadwheel60_ResetRIZAL(float x, float y, float desired_yaw_deg) {
	ODO_LastEncR = (int16_t) __HAL_TIM_GET_COUNTER(&htim3); // encoder kanan
	ODO_LastEncL = (int16_t) __HAL_TIM_GET_COUNTER(&htim1); // encoder kiri

	/*
	 * Ini inti penting:
	 * IMU raw tidak harus 0.
	 * desired_yaw_deg adalah heading yang ingin dianggap oleh program.
	 */
	ODO_YawOffsetDeg = DataYawIMU - desired_yaw_deg;

	PosX_World = x;
	PosY_World = y;

	RobotYaw = NormalizeDeg180_RIZAL(desired_yaw_deg);
	ODO_LastYawDeg = RobotYaw;
}

void BacaPosisiEncoderDeadwheel60_RIZAL(void) {
	int16_t encR_now = (int16_t) __HAL_TIM_GET_COUNTER(&htim3);
	int16_t encL_now = (int16_t) __HAL_TIM_GET_COUNTER(&htim1);

	int16_t d_tickR = (int16_t) (encR_now - ODO_LastEncR);
	int16_t d_tickL = (int16_t) (encL_now - ODO_LastEncL);

	ODO_LastEncR = encR_now;
	ODO_LastEncL = encL_now;

	float yaw_now_deg = Odo_GetRobotYawRIZAL();

	float dYaw_deg = NormalizeDeg180_RIZAL(yaw_now_deg - ODO_LastYawDeg);
	float dYaw_rad = dYaw_deg * ODO_DEG2RAD;

	float yaw_mid_deg = ODO_LastYawDeg + (0.5f * dYaw_deg);
	float yaw_mid_rad = yaw_mid_deg * ODO_DEG2RAD;

	ODO_LastYawDeg = yaw_now_deg;

	RobotYaw = yaw_now_deg;

	float sR = ((float) d_tickR) * ODO_R_MM_PER_TICK * ODO_R_DIR;
	float sL = ((float) d_tickL) * ODO_L_MM_PER_TICK * ODO_L_DIR;

	float aR = ODO_R_ANGLE_DEG * ODO_DEG2RAD;
	float aL = ODO_L_ANGLE_DEG * ODO_DEG2RAD;

	float rotR = (sinf(aR) * ODO_R_POS_Y_MM - cosf(aR) * ODO_R_POS_X_MM)
			* dYaw_rad;
	float rotL = (sinf(aL) * ODO_L_POS_Y_MM - cosf(aL) * ODO_L_POS_X_MM)
			* dYaw_rad;

	sR -= rotR;
	sL -= rotL;

	float Right_mm = (sL - sR) * 0.57735027f;
	float Forward_mm = sR + sL;

	PosX_World += (Right_mm * cosf(yaw_mid_rad))
			+ (Forward_mm * sinf(yaw_mid_rad));
	PosY_World += (-Right_mm * sinf(yaw_mid_rad))
			+ (Forward_mm * cosf(yaw_mid_rad));

	RobotYaw = NormalizeDeg180_RIZAL(yaw_now_deg);

	ODO_DBG_sR = sR;
	ODO_DBG_sL = sL;
	ODO_DBG_Right = Right_mm;
	ODO_DBG_Forward = Forward_mm;
	ODO_DBG_rotR = rotR;
	ODO_DBG_rotL = rotL;
	ODO_DBG_dYaw = dYaw_deg;
	ODO_DBG_YawNow = RobotYaw;
}

void SetWorldPosition_RIZAL(float x, float y, float h) {
	OdoDeadwheel60_ResetRIZAL(x, y, h);
}
