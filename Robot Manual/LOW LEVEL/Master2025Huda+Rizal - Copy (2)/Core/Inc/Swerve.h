/*
 * Swerve.h
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

#ifndef INC_SWERVE_H_
#define INC_SWERVE_H_

#include "main.h"
#include "myDEF.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern void JalanKeTitikEdit(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_biruu_valid(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_baru_Pindah_silo(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_baru_Eko(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_biruu(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void custom_delay_s(short delay);

extern void loopDelay(unsigned short ddd);

extern void RodaStopRemSemua(void);

extern  volatile uint32_t delayManual;

extern void JalanSudut(float speed, float sudut);
extern void RodaStopRemDepanAbs(void);
extern void JalanKeTitikSudutAbsolute(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void RodaNol_Abs(void);
extern void JalanKeTitikToSilo(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitikFromSilo(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);

extern void JalanKeTengah(float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKePilihKeranjang(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void tembok_Y(short keranjang, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_baru(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void Track_Ball_griper(float TargetH, float BatasError, char sFlag, int StartSpeed);

extern void Track_Ball_griper_depan(float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void Limit_depan_jalan();
extern void Koreksi_zilo(short zilo);

extern void RodaStopRemDepan(void);

extern int cek_nilai;
extern float cek_limit[15];

extern void CameraFront( float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void CameraFrontMid( float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik_K_baru_valid(short keranjang, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern int Track_Baru_Nasional();


extern int Track_Ball_griper_lifter_roda();

//VARIABEL BARU
extern volatile uint32_t time_out;
extern volatile uint32_t time_out1;
extern volatile uint32_t time_out2;

extern int Track_Ball_griper_lifter();
extern void Track_Ball_griper_lifter_v2();
extern int Track_Ball_griper_lifter_baru();
extern void pick_ball();


extern void CameraFrontGriper( float TargetH, float BatasError, char sFlag, int StartSpeed);


//data cam
volatile extern uint8_t id_object, index_object, index_near;
volatile extern uint32_t  center_y;
extern int center_x;
volatile extern uint8_t id_object_2, index_object_2, index_near_2, luas_2;
volatile extern uint32_t center_x_2, center_y_2;

extern int Track_Baru_Nasional();


extern  _Bool flag_putar_roda;

extern void KinematikDepan(float Vx, float Vy, float omega_deg);

extern void Kinematik(float Vx, float Vy, float omega_deg);
extern void Kinematik30Deg(float Vx, float Vy, float omega_deg);
extern void KinematikReverse(float Vy, float Vx, float omega_deg);
extern void IncrementSudutSwerve(float Sudut);
extern void KinematikSpeedOff(float Vx, float Vy, float omega_deg);
extern void RodaStop(void);
extern void RodaStopRemKananKiri(void);


extern void StopSpeed(int Time);

/*
 * RIZAL dynamic braking helper.
 * Dipakai untuk rem aktif setelah gerak otomatis/path planner.
 * Jangan mengganti RodaStop() global; panggil fungsi ini hanya jika butuh rem aktif.
 */
extern void RodaBrakeDynamicRIZAL(float gain, float max_brake, uint32_t brake_ms);
extern void RodaBrakeResetLastCommandRIZAL(void);
extern float R1_BRAKE_DBG_LastVx;
extern float R1_BRAKE_DBG_LastVy;
extern float R1_BRAKE_DBG_LastOmega;
extern float R1_BRAKE_DBG_CmdVx;
extern float R1_BRAKE_DBG_CmdVy;
extern float R1_BRAKE_DBG_CmdOmega;
extern uint32_t R1_BRAKE_DBG_LastMs;

extern void SetBatas (float *x, float batas);
extern void Kinematik30DegV2(float Vx, float Vy, float omega_deg);

//extern uint16_t data_analog1;
//extern uint16_t data_analog2;

extern float 	MaxSpeed, LimitErrorJ, LimitErrorH, LimitErrorTrace;
extern float	KPj,KDj, KPh, KDh;//, KIh=0;
extern float 	KPTrace,KDTrace;

extern void BacaPosisiEncoder(void);
extern void speed_Encoder(void);

extern volatile uint16_t sick_sensor_left_back;
extern volatile uint16_t sick_sensor_right_back;
extern volatile uint16_t sick_sensor_left;
extern volatile uint16_t sick_sensor_right;

extern void JalanKeTitikPitchRoll(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);

extern void JalanKeTembok_Biru(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTembok(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitik(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void JalanKeTitikRIZAL(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
void JalanKeTitikFinalHeadingRIZAL(float TargetX,
                                   float TargetY,
                                   float TargetH,
                                   float BatasJarak,
                                   float BatasHeading,
                                   int StartSpeed);
extern void JalanKeTitikHolonomic(float TargetX, float TargetY, float TargetH, float BatasError, char sFlag, int StartSpeed);
extern void RotasiHeadingAja(float TargetH, float Batas);

extern void data_analog();
extern void Parkir();
extern void KinematikSpeedOffParkir(float Vx, float Vy, float omega_deg);

extern void SetHeading(float h);
extern void SetWorldPosition_X_and_Y(float x, float y);
extern void SetWorldPosition(float x, float y, float h);
extern void SetWorldPosHeading0(float x, float y);
extern void ResetWorldPosition(void);
extern float targetH;

extern  _Bool flag_puter_roda;
extern float last_sudut_M1, last_sudut_M2, last_sudut_M3, last_sudut_M4;
extern float LastSpeed1, LastSpeed2, LastSpeed3;
extern short	SpeedEncA,SpeedEncB;
extern float PosX_World, PosY_World, RobotYaw, YawOffset;		// data posisi di lapangan
extern float PosX_robot, PosY_robot;
extern volatile unsigned char togglePosisi;



//Variable Spline
extern int titikX[10], titikY[10];
extern double inc, t, titikXS[100], titikYS[100], w[10], headingR[100];

extern uint16_t indekTitik;

extern double numX, numY, denum;

//Flag Counter Tembak
//extern long CounterStepTembak;
//extern uint8_t FlagTembak;
extern _Bool PaksaR;
extern _Bool FlagPaksa;
extern _Bool FlagPaksaTrace;

extern float ODO_DBG_sR;
extern float ODO_DBG_sL;
extern float ODO_DBG_Right;
extern float ODO_DBG_Forward;
extern float ODO_DBG_rotR;
extern float ODO_DBG_rotL;
extern float ODO_DBG_dYaw;
extern float ODO_DBG_YawNow;

void OdoDeadwheel60_ResetRIZAL(float x, float y, float desired_yaw_deg);
void BacaPosisiEncoderDeadwheel60_RIZAL(void);
float Odo_GetRobotYawRIZAL(void);
void SetWorldPosition_RIZAL(float x, float y, float h);

#endif /* INC_SWERVE_H_ */
