/*
 * USART.c
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */


#include "USART.h"
#include "stdio.h"
#include <string.h>

#include "main.h"
#include "cmsis_os2.h"
#include "Profiler.h"
//#include <Swerve.h>

extern osMessageQueueId_t uartTxQueueHandle;
//Variable
uint8_t volatile Rx_RS485[1];

volatile uint8_t	DataRemote1=0, DataRemote2=0, DataRemote3 = 0, DataRemote4 = 0,DataRemote5 = 0, DataRemote6 = 0 ;
//volatile int8_t	LX =  0, LY = 0, RX=0, RY = 0;

volatile uint8_t  Rx_Remote[1], Rx_IMU[1], Rx_VL[1],Rx_GUI[1];  //  creating a buffer of 10 bytes

uint16_t TOF1,TOF2;

uint8_t Target1GUI=0,Target2GUI=0,Target3GUI=0,Target4GUI=0;

_Bool FlagBuzzer = 0;

//Parsing IMU
//float DataYawIMU, DataPitchIMU, DataRollIMU;
uint8_t bufImu_0,bufImu_1,bufImu_2,bufImu_3;


void KirimUSART2(uint8_t data)
{
	USART2->DR = data;
	while(!(USART2->SR & USART_FLAG_TXE));
	USART2->SR &= ~USART_FLAG_TXE;
}

void KirimUSART3(uint8_t data)
{
//	USART3->DR = data;
//	while(!(USART3->SR & USART_FLAG_TXE));
//	USART3->SR &= ~USART_FLAG_TXE;
}

void KirimUSART6(uint8_t data)
{
	USART6->DR = data;
	while(!(USART6->SR & USART_FLAG_TXE));
	USART6->SR &= ~USART_FLAG_TXE;
}

void KirimUSART5(uint8_t data)
{
	UART5->DR = data;
	while(!(UART5->SR & USART_FLAG_TXE));
	UART5->SR &= ~USART_FLAG_TXE;
}


void sudut_pick_ball(float heading, uint8_t SlaveAddress)
{	uint8_t data_Roda[6], cksum, headingH, headingL;
	short headingS;
//  heading*=10.0f;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	cksum = (uint8_t)headingH ^ headingL;
	data_Roda[0] = 0XFF;
	data_Roda[1] = 0xFE;
	data_Roda[2] = 4;
	data_Roda[3] = headingH;
	data_Roda[4] = headingL;
	data_Roda[5] = cksum;
	//HAL_UART_Transmit(&huart2, (uint8_t*)data_Roda, 7, 20);

  	for(int a=0;a<=5;a++)
	{
		KirimUSART2(data_Roda[a]);
	}

}


int Blocking(uint8_t val)
{	uint8_t data_Roda[5], cksum;
	cksum = (uint8_t)27;
	data_Roda[0] = 0XFF;
	data_Roda[1] = 0xFE;
	data_Roda[2] = 0x90;
	data_Roda[3] = val;
	data_Roda[4] = cksum;
HAL_UART_Transmit(&huart2, (uint8_t*)data_Roda, 5, 20);
return 0;
}

//USART Roda
int SetKecepatanDanSudutRoda(signed char speed, float heading, uint8_t SlaveAddress)
{
//	uint8_t data_Roda[7], cksum, headingH, headingL;

	if( speed==-128) return 0;	// data kecepatan salah
	PROF_BEGIN(PROF_USART_SET_RODA);

	UartTxFrame_t buffer= {0};
	uint8_t cksum, headingH, headingL;
	short headingS ;
	heading*=10.0f;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	cksum = (uint8_t)speed ^ headingH ^ headingL;
	buffer.data[0] = 0XFF;
	buffer.data[1] = 0xFE;
	buffer.data[2] = SlaveAddress;
	buffer.data[3] = speed;
	buffer.data[4] = headingH;
	buffer.data[5] = headingL;
	buffer.data[6] = cksum;
	buffer.tick = osKernelGetTickCount();
	PROF_BEGIN(PROF_USART_QUEUE_PUT);
	(void) osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U);
	PROF_END(PROF_USART_QUEUE_PUT);
//	HAL_UART_Transmit(&huart3, (uint8_t*)data_Roda, 7, 20);



//  	for(int a=0;a<=6;a++)
//	{
//		KirimUSART3(data_Roda[a]);
//	}

PROF_END(PROF_USART_SET_RODA);
return 0;
}

int PneumaticMotor(int8_t speed, float heading, uint8_t led)
{
//	uint8_t data_Roda[7], cksum, headingH, headingL;
	UartTxFrame_t buffer= {0};
		uint8_t cksum, headingH, headingL, LED;
	short headingS ;
	heading*=10.0f;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	LED = led != 0? 1:0;
	cksum = (uint8_t)speed ^ headingH ^ headingL ^ LED;
	buffer.data[0] = 0XFF;
	buffer.data[1] = 0xFE;
	buffer.data[2] = IdMotorDcTongkat;
	buffer.data[3] = speed;
	buffer.data[4] = headingH;
	buffer.data[5] = headingL;
	buffer.data[6] = LED;
	buffer.data[7] = cksum;

	buffer.tick = osKernelGetTickCount();
		(void) osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U);

//  	for(int a=0;a<=6;a++)
//	{
//		KirimUSART3(data_Roda[a]);
//	}
return 0;
}

//USART Roda
int LifterUtama(float heading, uint8_t mode_)
{
    UartTxFrame_t buffer= {0};
    uint8_t cksum;
    uint8_t headingH, headingL;
    int headingS;

    headingS = (int)heading;

//    if (headingS > 4242) headingS = 4242;
//    if (headingS < 0)    headingS = 0;

    headingH = (uint8_t)((headingS >> 8) & 0xFF);
    headingL = (uint8_t)(headingS & 0xFF);

    uint8_t mode = mode_ != 0 ? 1 : 0; // 0 = lambat, 1 = cepat

    cksum = (uint8_t)(headingL ^ headingH ^ mode);

    buffer.data[0] = 0xFF;
    buffer.data[1] = 0xFE;
    buffer.data[2] = IdLifterUtama;   // harus 0x04 kalau parser hardcode 0x04
    buffer.data[3] = headingL;
    buffer.data[4] = headingH;
    buffer.data[5] = mode;
    buffer.data[6] = cksum;
//    buffer.data[7] = 0x00;            // penting kalau queue sender selalu kirim 8 byte

    buffer.tick = osKernelGetTickCount();

    if (osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U) != osOK)
    {
        // optional: counter queue fail
    }

    return 0;
}

int LifterKFs(float heading)
{
	//	uint8_t data_Roda[7], cksum, headingH, headingL;
	UartTxFrame_t buffer= {0};
		uint8_t cksum, headingH, headingL;
	short headingS ;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	cksum = (uint8_t)0 ^ headingH ^ headingL;
	buffer.data[0] = 0XFF;
	buffer.data[1] = 0xFE;
	buffer.data[2] = IdLifterKfs;
	buffer.data[3] = 0;
	buffer.data[4] = headingH;
	buffer.data[5] = headingL;
	buffer.data[6] = cksum;
	buffer.tick = osKernelGetTickCount();
		(void) osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U);
//	HAL_UART_Transmit(&huart3, (uint8_t*)data_Roda, 7, 20);

//  	for(int a=0;a<=6;a++)
//	{
//		KirimUSART3(data_Roda[a]);
//	}
return 0;
}

int DegKFS(float heading)
{
//	uint8_t data_Roda[7], cksum, headingH, headingL;
	UartTxFrame_t buffer= {0};
		uint8_t cksum, headingH, headingL;
	short headingS ;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	cksum = (uint8_t)0 ^ headingH ^ headingL;
	buffer.data[0] = 0XFF;
	buffer.data[1] = 0xFE;
	buffer.data[2] = IdDegSudut;
	buffer.data[3] = 0;
	buffer.data[4] = headingH;
	buffer.data[5] = headingL;
	buffer.data[6] = cksum;
	buffer.tick = osKernelGetTickCount();
		(void) osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U);
//  	for(int a=0;a<=6;a++)
//	{
//		KirimUSART3(data_Roda[a]);
//	}
return 0;
}



int Pneumatic(float pn1, float pn2, float pn3)
{
//	uint8_t data_Roda[7];
	UartTxFrame_t buffer= {0};
	buffer.data[0] = 0XFF;
	buffer.data[1] = 0xFE;
	buffer.data[2] = 0x08;
	buffer.data[3] = pn1;
	buffer.data[4] = pn2;
//	data_Roda[5] = pn1;
	buffer.data[5] = pn3;
	buffer.data[6] = 27;
	buffer.tick = osKernelGetTickCount();
		(void) osMessageQueuePut(uartTxQueueHandle, &buffer, 0U, 0U);
//  	for(int a=0;a<=6;a++)
//	{
//		KirimUSART3(data_Roda[a]);
//	}
return 0;
}














void send_pneumatic(short pneumatic){
	uint8_t data[5];
	uint8_t cksum;
	cksum = 27;
	data[0] = 0xFF;
	data[1] = 0xFE;
	data[2] = 0x06;
	data[3] = pneumatic;
	data[4] = cksum;
	HAL_UART_Transmit(&huart2, (uint8_t*)data, 5, 20);
}

int SetKecepatanSpiner(float Spin1, float Spin2, float Spin3, uint8_t Flag)
{	uint8_t data_Roda[11], cksum, SpinH1, SpinL1, SpinH2, SpinL2, SpinH3, SpinL3, DynH1, DynL1, DynH2, DynL2;
	short SpinS1, SpinS2, SpinS3, DynS1, DynS2 ;
	Spin1*=10.0f;
	Spin2*=10.0f;
	Spin3*=10.0f;
//	Dyn1*=10.0f;
//	Dyn2*=10.0f;

	SpinS1 = Spin1;
	SpinH1 = SpinS1>>8;
	SpinL1 = SpinS1&255;
	SpinS2 = Spin2;
	SpinH2 = SpinS2>>8;
	SpinL2 = SpinS2&255;
	SpinS3 = Spin3;
	SpinH3 = SpinS3>>8;
	SpinL3 = SpinS3&255;

//	DynS1 = Dyn1;
//	DynH1 = DynS1>>8;
//	DynL1 = DynS1&255;
//	DynS2 = Dyn2;
//	DynH2 = DynS2>>8;
//	DynL2 = DynS2&255;

	cksum = (uint8_t)27;
	data_Roda[0] = 0XFF;
	data_Roda[1] = 0xFE;
	data_Roda[2] = 0x05;
	data_Roda[3] = SpinH1;
	data_Roda[4] = SpinL1;
	data_Roda[5] = SpinH2;
	data_Roda[6] = SpinL2;
	data_Roda[7] = SpinH3;
	data_Roda[8] = SpinL3;
	data_Roda[9] = Flag;
	data_Roda[10] = cksum;
//	data_Roda[10] = DynH1;
//	data_Roda[11] = DynL1;
//	data_Roda[12] = DynH2;
//	data_Roda[13] = DynL2;
//
//	data_Roda[14] = cksum;
	HAL_UART_Transmit(&huart2, (uint8_t*)data_Roda, 11, 50);

return 0;
}



void SetKpKd(uint8_t Kp, uint8_t Kd, uint8_t SlaveAddress)
{	uint8_t data_Roda[7], cksum = 1 ^ Kp ^ Kd;
	data_Roda[0] = 0XFF;
	data_Roda[1] = 0xFE;
	data_Roda[2] = SlaveAddress;
	data_Roda[3] = 1;
	data_Roda[4] = Kp;
	data_Roda[5] = Kd;
	data_Roda[6] = ~cksum;
	HAL_UART_Transmit(&huart2, (uint8_t*)data_Roda, 7, 20);
}


void RakOnly( uint8_t IdSlave, short data){

  uint8_t dataRak[7],headingH, headingL, cksum;
  short datas;
  datas=data;
  headingH = datas>>8;
  headingL = datas&255;
  cksum = (uint8_t)headingH^headingL;
  dataRak[0] = 0xFF;
  dataRak[1] = 0xFE;
  dataRak[2] = IdSlave;
  dataRak[3] = headingH;
  dataRak[4] = headingL;
  dataRak[5] = cksum;
  dataRak[6] = cksum;

  HAL_UART_Transmit(&huart2, (uint8_t*)dataRak, 7, 20);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Turret Control
int SetSlaveTurretControl(uint8_t IdSlave,uint8_t Flag, float Data)
{
	uint8_t data_Roda[8], cksum, headingH, headingL;
	short DataS ;
	Data*=10.0f;
	DataS = Data;
	headingH = DataS>>8;
	headingL = DataS&255;
	cksum = (uint8_t)Flag ^ headingH ^ headingL;
	data_Roda[0] = 0XFF;
	data_Roda[1] = 0xFE;
	data_Roda[2] = IdSlave;
	data_Roda[3] = Flag;
	data_Roda[4] = headingH;
	data_Roda[5] = headingL;
	data_Roda[6] = cksum;
	data_Roda[7] = cksum;

	HAL_UART_Transmit(&huart2, (uint8_t*)data_Roda, 8, 20);
return 0;
}


void ParsingDataJoystick(uint8_t c)
{		static uint8_t sBuffer[7], sCounter=0, sflag=0;
//		uint8_t pesan[20];
		if (sflag==0)
		{	if(c==0xFF) sflag=1;	// header 1
			else sflag=0;
		}
		else if(sflag==1)
		{	if(c==0xFE)
			{	sflag=2;	// header 2
				sCounter=0;
			}
			else	sflag=0;	// reset flag
		}
		else if(sflag==2)
		{	sBuffer[sCounter] = c;
			sCounter++;
			if(sCounter==7)			// data kedua diterima
			{	if( sBuffer[6]==(uint8_t)(sBuffer[0]+sBuffer[1])) // hitung checksum
				{	DataRemote1= sBuffer[0];	// data berhasil diterima
					DataRemote2= sBuffer[1];
//					LY= sBuffer[2]-100;
//					LX= (sBuffer[3]-100)*-1;
//					RY= sBuffer[4]-100;
//					RX= (sBuffer[5]-100)*-1;

					}
				sflag = 0;	 // --> reset flag kembali
			}
		}

}


//Parsing IMU
//float DataYawIMU;

//void ParsingDataIMU(uint8_t c){
//    static uint8_t sBuffer[4], sCounter=0, sflag=0;
//
//    if(sflag==0)
//    {
//        if(c==0xFF) sflag=1;
//    }
//    else if(sflag==1)
//    {
//        if(c==0xFE)
//        {
//            sflag=2;
//            sCounter=0;
//        }
//        else sflag=0;
//    }
//    else if(sflag==2)
//    {
//        sBuffer[sCounter] = c;
//        sCounter++;
//
//        if(sCounter==3)
//        {
//            uint8_t calculatedChecksum = sBuffer[0] + sBuffer[1] ;
//
//            if(sBuffer[2] == calculatedChecksum)
//            {
//                uint16_t azimuth = (sBuffer[0] << 8) | sBuffer[1];
//                DataYawIMU = 360-(azimuth / 10.0f);
//            }
//            sflag=0;
//        }
//    }
//
//}

