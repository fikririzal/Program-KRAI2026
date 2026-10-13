/*
 * dyn.c
 *
 *  Created on: Jul 1, 2026
 *      Author: USER
 */


#include "dyn.h"
#include "string.h"
#include "MyDEF.h"
#include "Profiler.h"

void MoveServo(int servoID, int Position)
{
    PROF_BEGIN(PROF_DYN_MOVE_SERVO);
	//length = jumlah parameter + 2
	//Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
	//(header,header,ID,length,write,paramter,parameter1,parameter2,parameter3,notchecksum)
	/*https://smartgrasp.wordpress.com/dynamixel-rx-28-rs-485-arduino*/
  uint8_t Position_H = Position >> 8;  //same as /256 and truncating
  uint8_t Position_L = Position % 256;
  uint8_t notchecksum = ~(servoID + 0x05 + 0x03 + 0x1E + Position_L + Position_H);
	uint8_t data[10];//, i;
	//uint8_t notchecksum = ~(servoID + 0x08 + 0x03 + 0x1E + Position_L + Position_H + 0x20 + 0x3FF + 0x3FF);

  //delay_ms(1);
	//uint8_t data[12]={(0xFF),(0xFF),servoID,(0x05),(0x08),(0x1E),Position_L,Position_H,(0x20),(0x3FF),(0x3FF),notchecksum};
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = 0x1e;
	data[6] = Position_L;
	data[7] = Position_H;
	data[8] = notchecksum;
//	for (uint8_t i=0;i<9;i++)
//	{
	HAL_UART_Transmit(&huart6, (uint8_t*)data,9, 20);
	osDelay(10);
	HAL_UART_Transmit(&huart6, (uint8_t*)data,9, 20);
		osDelay(10);
		HAL_UART_Transmit(&huart6, (uint8_t*)data,9, 20);
			osDelay(10);
    PROF_END(PROF_DYN_MOVE_SERVO);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, strlen((const char*)data));

	//HAL_Delay(3);
//	}
//	DynamixelTxOff;	// mode transmisi non aktif;
}

void MoveDrive(int servoID, int Speed)
{
	//length = jumlah parameter + 2
	//Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
	//(header,header,ID,length,write,paramter,parameter1,parameter2,parameter3,notchecksum)
	/*https://smartgrasp.wordpress.com/dynamixel-rx-28-rs-485-arduino*/
  uint8_t Position_H = Speed >> 8;  //same as /256 and truncating
  uint8_t Position_L = Speed % 256;
  uint8_t notchecksum = ~(servoID + 0x05 + 0x03 + 0x20 + Position_L + Position_H);
	uint8_t data[10];//, i;
	//uint8_t notchecksum = ~(servoID + 0x08 + 0x03 + 0x1E + Position_L + Position_H + 0x20 + 0x3FF + 0x3FF);

  //delay_ms(1);
	//uint8_t data[12]={(0xFF),(0xFF),servoID,(0x05),(0x08),(0x1E),Position_L,Position_H,(0x20),(0x3FF),(0x3FF),notchecksum};
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = 0x20;//0x20
	data[6] = Position_L;
	data[7] = Position_H;
	data[8] = notchecksum;
//	for (uint8_t i=0;i<9;i++)
//	{
	//HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen((const char*)data), 200);
	HAL_UART_Transmit(&huart1, (uint8_t*)data, 9, 1);
  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, strlen((const char*)data));


//	}
//	DynamixelTxOff;	// mode transmisi non aktif;
}

void EnableTorque(int servoID, uint8_t Enable)
{
	uint8_t notchecksum = ~(servoID + 0x05 + 0x03 + 0x46 + Enable);

	uint8_t data[8];//, i;

	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x04;
	data[4] = 0x03;
	data[5] = 0x46;//18
	data[6] = Enable;
	data[7] = notchecksum;
	HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((const char*)data), 100);
}

void SetAngleLimitCCW(int servoID, short Angle)
{
	uint8_t Position_H = Angle >> 8;  //same as /256 and truncating
  uint8_t Position_L = Angle % 256;
  uint8_t notchecksum = ~(servoID + 0x05 + 0x03 + 8 + Position_L + Position_H);

	uint8_t data[10];//, i;

	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = 8;//18
	data[6] = Position_L;
	data[7] = Position_H;
	data[8] = notchecksum;

	HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((const char*)data), 100);
}

// pilihan nilai KP: 8 16 32 64 128 (nilai default 32)
void SetServoKp(int servoID, uint8_t Kp)
{
	uint8_t notchecksum = ~(servoID + 0x04 + 0x03 + 28 + Kp);
	char data[8];//, i;

  //delay_ms(1);
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x04;
	data[4] = 0x03;
	data[5] = 28;
	data[6] = Kp;
	data[7] = notchecksum;
	//for (i=0;i<8;i++)
	HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 100);
  HAL_Delay(3);
//	DynamixelTxOff;	// mode transmisi non aktif;
}

// pilihan nilai 0-1023 ()
void SetServoTorqueLimit(int servoID, int Torque)
{
  uint8_t Torque_H = Torque >> 8;  //same as /256 and truncating
  uint8_t Torque_L = Torque % 256;
  uint8_t notchecksum = ~(servoID + 0x05 + 0x03 + 34 + Torque_H + Torque_L);
	char data[9];//, i;

  //delay_ms(2);
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = 34;
	data[6] = Torque_L;
	data[7] = Torque_H;
	data[8] = notchecksum;
	//for (i=0;i<9;i++) HAL_UART_Transmit(&huart1, (uint8_t*) data[i], 1, 100);
	HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 100);
  HAL_Delay(3);
//	DynamixelTxOff;	// mode transmisi non aktif;
}

void SetSpeed(int servoID, uint16_t Speed)
{
    PROF_BEGIN(PROF_DYN_SET_SPEED);
	uint8_t Speed_L = Speed & 0xFF;
	uint8_t Speed_H = (Speed >> 8) & 0xFF;
	uint8_t checksum = ~(servoID + 0x05 + 0x03 + 0x20 + Speed_L + Speed_H);
	uint8_t data[9];

	//delay_ms(2);
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = servoID;
	data[3] = 0x05;
	data[4] = 0x03;
	data[5] = 0x20;
	data[6] = Speed_L;
	data[7] = Speed_H;
	data[8] = checksum;
	//for (i=0;i<9;i++) HAL_UART_Transmit(&huart1, (uint8_t*) data[i], 1, 100);
	HAL_UART_Transmit(&huart6, data, 9, 100);
    PROF_END(PROF_DYN_SET_SPEED);
	//HAL_Delay(3);
//	DynamixelTxOff;	// mode transmisi non aktif;
}
