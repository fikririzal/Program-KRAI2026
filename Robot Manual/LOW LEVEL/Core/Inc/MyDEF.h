/*
 * MyDEF.h
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

#ifndef INC_MYDEF_H_
#define INC_MYDEF_H_

#include "Joystick.h"

#define id_posisi_robotROS 99


#define SetBit(x,y) 	x->BSRR = (1<<y)
#define ClrBit(x,y) 	x->BRR  = (1<<y)
#define Pin(Port,x)	  (Port->IDR&(1<<x))

#define SetA(y) 		GPIOA->BSRR = (1<<y)
#define SetB(y) 		GPIOB->BSRR = (1<<y)
#define SetC(y) 		GPIOC->BSRR = (1<<y)
#define SetD(y) 		GPIOD->BSRR = (1<<y)
#define SetE(y) 		GPIOE->BSRR = (1<<y)
#define SetF(y) 		GPIOF->BSRR = (1<<y)
#define SetG(y) 		GPIOG->BSRR = (1<<y)

#define ClrA(y) 		GPIOA->BRR  = (1<<y)
#define ClrB(y) 		GPIOB->BRR  = (1<<y)
#define ClrC(y) 		GPIOC->BRR  = (1<<y)
#define ClrD(y) 		GPIOD->BRR  = (1<<y)
#define ClrE(y) 		GPIOE->BRR  = (1<<y)
#define ClrF(y) 		GPIOF->BRR  = (1<<y)
#define ClrG(y) 		GPIOG->BRR  = (1<<y)


// mengakses LED pada modul BLDC LED
#define	LedHallA(o)	(o ? SetB(5) : ClrB(5))
#define	LedHallB(o)	(o ? SetB(6) : ClrB(6))
#define	LedHallC(o)	(o ? SetB(7) : ClrB(7))

#define	SetLedHallA		SetBit(GPIOB,5)
#define	SetLedHallB		SetBit(GPIOB,6)
#define	SetLedHallC		SetBit(GPIOB,7)
#define	ClrLedHallA		ClrBit(GPIOB,5)
#define	ClrLedHallB		ClrBit(GPIOB,6)
#define	ClrLedHallC		ClrBit(GPIOB,7)

//Enable untuk  motor
#define EnableA(o)	(o ? SetB(12) : ClrB(12))
#define EnableB(o)	(o ? SetB(14) : ClrB(14))
#define EnableC(o)	(o ? SetB(15) : ClrB(15))

// konstanta konversi radian dan derajat
#define RAD2DEG     (float) 57.295779513082320876798154814105
#define DEG2RAD     (float) 0.01745329251994329576923690768489

//ID RS 485
#define IdRoda1 		1
#define IdRoda2 		2
#define IdRoda3			3
#define IdRoda4			4

#define IdLifterVertikal		0x04
#define IdLifterHorizontal    0x05

#define OffsetYawLifter 	2280/10.0f//3795//3756
#define IdYawLifter 6

#define OffsetDepanKiri1  (1050.0f/10.0f)+180.0	//3485	1720
#define	OffsetDepanKiri2  (1450.0f/10.0f)+180.0
#define	OffsetDepanKiri3  (1950.0f/10.0f)+0.0

#define Offset1  (600.0f/10.0f)+180.0	//3485	1720
#define	Offset2  (1400.0f/10.0f)+0.0
#define	Offset3  (695.0f/10.0f)+180.0


#define IdLifterUtama 0x04
#define IdLifterKfs 0x05
#define IdDegSudut 0x06
#define IdMotorDcTongkat 0x07
#define IdPneumatic 0x08

#define gripTongkatAmbil 1800
#define gripTongkat90 0
#define gripTongkat0 2650





























#define	Offset4  (1800.0f/10.0f)+180.0

//Kecepatam
#define minSpeed	70
#define maxSpeed	125
#define minSteer	15 //10
#define maxSteer	20 //20

#define Idsudutpick_ball		4

#define IdControlPitch		0x04
#define IdSlaveTurret1 		0x04
#define IdSlaveTurret2 		0x05
#define IdSlaveSpinnerR 	0x05
#define IdSlaveSpinnerL		0x06
#define IdSlaveServo			0x07

//Control Pitch
#define StopPitch				0x20
#define ManualPitch			0x21
#define OtomatisPitch		0x22
#define OtomatisGrip		0x23
/*
//Flag Flag Control Turret
//Data Turret1
#define StopTurret			0x20
#define ManualYaw 			0x21
#define ManualPitch 		0x22
#define OtomatisYaw 		0x23
#define OtomatisPitch 	0x24
#define Pnematic				0x25
#define SetSpeedYaw			0x26
#define SetSpeedPitch		0x27
//Data Turret2
#define OtomatisAngkat	0x20
#define SetSpeedAngkat	0x21
*/





#define IdDynPitch	0x01
#define IdDynYaw	0x02
#define IdDynGrip	0x03


//vision
#define IdDynPitch_vision	0x01
#define IdDynYaw_vision	0x02






#define OffsetYawTurret 	1150//3795//3756
#define OffsetYawTurretShot	-3110+OffsetYawTurret//718//3077//-1960

#define OffsetPitchTurret 206.5 //432
#define OffsetPitchGrip	 	92.5
#define Idslave_rak 0x80
#define Naik 0x81
#define Turun 0x82
#define Netral 0x83


#define NoneTrace 0x20
#define TraceDepan 0x21
#define TraceBelakang 0x22
#define TraceKanan 0x23
#define TraceKiri 0x24
#define TraceParkir 0x25

#define OpenServo		0x01
#define CloseServo	0x02

#define	RodaCanId			CanIdMaster

//#ifndef 	int8_t
//#define		int8_t		signed char
//#endif
//#ifndef 	uint8_t
//#define		uint8_t		unsigned char
//#endif
//#ifndef 	int16_t
//#define		int16_t		signed short
//#endif
//#ifndef 	uint16_t
//#define		uint16_t	unsigned short
//#endif
//#ifndef 	int32_t
//#define		int32_t		signed long
//#endif
//#ifndef 	uint32_t
//#define		uint32_t	unsigned long
//#endif

#define r  55
#define pi 3.1428571f



//#define	ResolusiEncoder		400.0f		// quadratur
//#define EncoderConstant		DiameterEncoder*PI/ResolusiEncoder/2.0f
#define	PI								3.1415926535897932384626433832795f
#define	Deg2Rad						PI/180.0f
#define	Rad2Deg						180.0f/PI

#define	DiameterEncoder		49.864f
#define	JarakEnceder			427.024f
#define	ResolusiEncoder		8112.0f		// quadratur
#define EncoderConstant		DiameterEncoder*PI/ResolusiEncoder/2.0f
//#define	SamplingTime			0.01f
#define	DiameterRoda			150				// dalam mm
// model 3 roda
#define	RadiusRobot				412				// dalam mm
#define	AkarTiga					1.7320508075688773f
#define Sin45             0.7071f
#define	Sin30							0.5f
#define	Sin60							0.866025404f
#define	Cos30							0.866025404f
#define	Cos60							0.5f


#define	servoID_0	(uint8_t)0x00	// lengan tembak naik turun
#define	servoID_1	(uint8_t)0x01	// gripper tembak kiri
#define	servoID_2	(uint8_t)0x02	// gripper tembak kanan
#define	servoID_3	(uint8_t)0x03	// gripper kicker kanan
#define	servoID_4	(uint8_t)0x04	// gripper kicker kiri
#define	servoID_5	(uint8_t)0x05	// gripper tembak kanan
#define	servoID_6	(uint8_t)0x06	// gripper tembak kanan
#define	servoID_7	(uint8_t)0x07	// gripper tembak kanan
#define	servoID_8	(uint8_t)0x08	// gripper tembak kanan



//Data Analog

#define	PS_L3				BTN_L3
#define	PS_L2				BTN_L2
#define	PS_L1				BTN_L1
#define	PS_Up				DPAD_ATAS
#define	PS_Down			DPAD_BAWAH
#define	PS_Left			DPAD_KIRI
#define	PS_Right		DPAD_KANAN
#define	PS_Select		MISC_SHARE

#define	PS_R3				BTN_R3
#define	PS_R2				BTN_R2
#define	PS_R1				BTN_R1
#define	PS_Triangle BTN_SEGITIGA
#define	PS_Cross		BTN_SILANG
#define	PS_Circle		BTN_LINGKARAN
#define	PS_Square		BTN_KOTAK
#define	PS_Start		MISC_OPTIONS
#define PS_Logo			MISC_PS

//Data Analog
#define PS_LYMin		JOYS_L_ATAS
#define PS_LYPlus		JOYS_L_BAWAH
#define PS_LXPlus		JOYS_L_KANAN
#define PS_LXMin		JOYS_L_KIRI

#define PS_RYMin		JOYS_R_ATAS
#define PS_RYPlua		JOYS_R_BAWAH
#define PS_RXPlus		JOYS_R_KANAN
#define PS_RXMin		JOYS_R_KIRI

#define PushButton6		HAL_GPIO_ReadPin(GPIOD, Limit_belakang_kanan_Pin)
#define PushButton5 	HAL_GPIO_ReadPin(GPIOD, Aux2_Pin)
#define PushButton2 	HAL_GPIO_ReadPin(GPIOD, Aux1_Pin)
#define PushButton4 	HAL_GPIO_ReadPin(GPIOD, Aux3_Pin)
#define PushButton1 	HAL_GPIO_ReadPin(GPIOD, Button1_Pin)
#define PushButton3 	HAL_GPIO_ReadPin(GPIOD, Button3_Pin)



//
#define FastMode	0x01
#define SlowMode	0x02
#define SportMode	0x03
#define Kunci	0x200



#define FromStart_Depan 1
#define FromStart_AtasKanan 2
#define FromStart_BawahKanan 3
#define FromStart_AtasKiri  4
#define FromStart_BawahKiri 5
#define FromRitry_ 6




//Max 2000
// Pengaturan nilai PWM ---------------
#define	SetPwm1A(x)			TIM1->CCR1 = x
#define	SetPwm1B(x)			TIM1->CCR2 = x
#define	SetPwm2A(x)			TIM1->CCR3 = x
#define	SetPwm2B(x)			TIM1->CCR4 = x
#define	SetPwm3A(x)			TIM8->CCR3 = x
#define	SetPwm3B(x)			TIM8->CCR4 = x
#define	SetPwm4A(x)			TIM8->CCR1 = x
#define	SetPwm4B(x)			TIM8->CCR2 = x

#define MaxPWM	5000
#define MinPWM	-MaxPWM



//Online
#define A 0	//FIX
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define G 6
#define H 7
#define I 8	//FIX
#define J 9
#define K 10

typedef struct
{
	float sudut;
	signed char speed;

}roda;


union   Data16Bit
{   int8_t      d8s[2];     // 2 buah data signed char
    uint8_t     d8u[2];     // 4 buah data unsigned char
    int16_t     d16s;       // 1 buah data signed short integer
    uint16_t    d16u;       // 1 buah data unsigned short integer
}__attribute__ ((__packed__));

union   Data32Bit
{   int8_t      d8s[4];     // 4 buah data signed char
    uint8_t     d8u[4];     // 4 buah data unsigned char
    int16_t     d16s[2];    // 2 buah data signed short integer
    uint16_t    d16u[2];    // 2 buah data unsigned short integer
    int32_t     d32s;       // 1 buah data signed long integer
    uint32_t    d32u;       // 1 buah data unsigned long integer
    float       df;         // 1 buah data float
}__attribute__ ((__packed__));


#endif /* INC_MYDEF_H_ */
