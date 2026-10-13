/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include "myDEF.h"
//#include "ee.h"
//extern	CAN_RxHeaderTypeDef myCanRxHeader;
//uint8_t	CanRxData[8];
extern volatile unsigned char NilaiHallSensor;
//extern volatile unsigned long CounterPosisi;
volatile unsigned char	flagOverCurrent=0;
extern 	signed short PWM;
//extern 	int AMS_5600_BacaSudut(void);
extern 	void SetSudutRoda(short sudut);

void  	Komutasi(void);
void Motor(int Speed);

extern 	UART_HandleTypeDef huart3;
extern 	UART_HandleTypeDef huart2;
//extern 	short offsetSudutRoda;
extern 	unsigned char KP, KD;
extern unsigned char KI; 
extern 	short TargetSudut, posisi;  
uint8_t sudut[20];
short 	sudutRoda;
//extern 	uint8_t CanTxData[8];
//extern 	CAN_TxHeaderTypeDef myCanTxHeader;
//extern 	uint32_t TxMailBox;
extern 	volatile char	flagCanRequest;
extern 	volatile uint8_t	flagOverTemperature;
extern 	uint32_t adcTemp;
uint16_t dataGlobal;
volatile uint8_t RxData[1];
//extern 	uint8_t 	EeKpKd[2], EeOffsetSudutRoda[2];	

void KontrolPutar(void);
#define LimitErrorSudut 	150		// 15 derajat
#define CenterPWM					250

//AMS5600 PWM INPUT
#define TIMCLOCK 72000000
#define PRESCALAR 72

uint32_t IC_Val1=0;
uint32_t IC_Val2=0;
uint32_t Difference=0;

uint8_t Is_First_Captured = 0;
float Frequency=0;
float Duty=0;

#if RodaId==Slave1
	uint16_t DataMin = 785 , DataMax = 24825; 
#elif RodaId==Slave2
		uint16_t DataMin = 775 , DataMax = 24880; 
#elif RodaId==Slave3
		uint16_t DataMin = 760 , DataMax = 25106; 
		#elif RodaId==Slave4
		uint16_t DataMin = 760 , DataMax = 25006; 
#endif

extern short KirimSudut;

short PersamaanPWM=0;
short BuffPWM=0;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
uint16_t DataSudut(uint16_t Data);
void HitungDataUsart3(uint8_t c);
extern uint8_t	SetBatasPwmRoda(short PwmRoda);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	HAL_UART_Receive_IT(&huart3, (uint8_t*) RxData, 1);
	HitungDataUsart3(RxData[0]);
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
short CekBuffer;
short Sudut;
void HitungDataUsart3(uint8_t c)
{
	static uint8_t sBuffer[4], sCounter=0, sflag=0, dataSpeed[5];	
	uint8_t dataSudut, sH, sL;
	if (sflag==0)
	{	if(c==0xFF) sflag=1;	// header 1
		else	sflag=0;	// reset flag	
	}	
	
	else if(sflag==1)
	{	if(c==0xFE) sflag=2; //header 2
		else	sflag=0;	// reset flag	
	}	

	else if(sflag==2)
	{	if(c==RodaId) { sflag=3; sCounter=0;}	// Slave id
		else	sflag=0;	// reset flag	
	}

	else if(sflag==3)
	{	sBuffer[sCounter] = c;
		sCounter++;
		if(sCounter==4)			// data ke-- diterima checksum
		{	if( sBuffer[3]==(uint8_t)(sBuffer[0]^sBuffer[1]^sBuffer[2])) // hitung checksum
			{	
				if(sBuffer[0]!=0x80) {
					SetSudutRoda((short)(sBuffer[1]<<8) + sBuffer[2]);
					dataSpeed[0] = 0x80; //-128
					dataSpeed[1] = 0xFF;
					dataSpeed[2] = sBuffer[0];
					dataSpeed[3] = (sBuffer[0]^0xFF);
					dataSpeed[4] = (sBuffer[0]^0xFF);
					
					
					CekBuffer = (short)((sBuffer[1]<<8) + sBuffer[2]);
					
					#if RodaId==Slave1 || RodaId==Slave2
						HAL_UART_Transmit(&huart1, (uint8_t*)dataSpeed, 5, 20);	// kirim kecepatan roda pada range -127 s/d 127
					#elif RodaId==Slave4||RodaId==Slave3
						HAL_UART_Transmit(&huart1, (uint8_t*)dataSpeed, 5, 20);	// kirim kecepatan roda pada range -127 s/d 127
					#endif
					
										
				}
			}
			
			else if(sBuffer[3]==(uint8_t)~(sBuffer[0]^sBuffer[1]^sBuffer[2]))
			{
				if(sBuffer[0] == 0x01){
					KP = sBuffer[1];
					KD = sBuffer[2];
				}
				else if(sBuffer[0] == 0x02) {
					if(sBuffer[1] == 0x03) {
						dataSudut =(short)DataSudut(Difference);
						sBuffer[0] = RodaId;
						sBuffer[1] = 1;
						sBuffer[2] = 2;
						sBuffer[3] = sBuffer[1]^sBuffer[2];
						HAL_GPIO_WritePin(GPIOB, RS485_Pin, GPIO_PIN_SET); //RS485Transmit
						for(uint8_t i= 0; i<10; i++)
						{
							HAL_UART_Transmit(&huart3, (uint8_t*)sBuffer, 4, 20);
						}
						SetLedHallA;
						HAL_Delay(50);
						HAL_GPIO_WritePin(GPIOB, RS485_Pin, GPIO_PIN_RESET); //RS485Receive	
					}
				}
//			sprintf(pesan,"r%d %d\n", sBuffer[0], sBuffer[1]); 
//			HAL_UART_Transmit_DMA(&huart3, pesan, strlen(pesan));
			}
			sflag = 0;	 // --> reset flag kembali 
		}
	}		
	
}


//Hasil Kalibrasi
#if RodaId==Slave1
	#define sudut0  0.0f
	#define sudut90 91.0f
	#define sudut180 180.0f
	#define sudut270 268.7f
	#define sudut360 359.8f
#elif RodaId==Slave2
	#define sudut0  0.0f
	#define sudut90 89.7f
	#define sudut180 179.4f
	#define sudut270 269.4f
	#define sudut360 359.9f
#elif RodaId==Slave3
	#define sudut0  0.0f
	#define sudut90 89.5f
	#define sudut180 179.5f
	#define sudut270 270.6f
	#define sudut360 359.8f
#elif RodaId==Slave4
	#define sudut0  0.0f
	#define sudut90 89.5f
	#define sudut180 179.5f
	#define sudut270 270.6f
	#define sudut360 359.8f
#endif

float ProgramKablirasiSudutSwerve(short SudutEnc)
{
	//Lagrange Polynomialll
	float SudutEncoder = (float)(SudutEnc/10.0f);
	float OutSudut;
	if(SudutEncoder>=0 && SudutEncoder<=180) 
	{      
		OutSudut = 90.0f*((SudutEncoder-sudut0)*(SudutEncoder-sudut180))/((sudut90-sudut0)*(sudut90-sudut180));
		OutSudut += 180.0f*((SudutEncoder-sudut0)*(SudutEncoder-sudut90))/((sudut180-sudut0)*(sudut180-sudut90));
		if (OutSudut<0) OutSudut+=360;
	}
	else
	{  
		OutSudut = 180.0f*((SudutEncoder-sudut270)*(SudutEncoder-sudut360))/((sudut180-sudut270)*(sudut180-sudut360));
		OutSudut+= 270.0f*((SudutEncoder-sudut180)*(SudutEncoder-sudut360))/((sudut270-sudut180)*(sudut270-sudut360));
		OutSudut+= 360.0f*((SudutEncoder-sudut180)*(SudutEncoder-sudut270))/((sudut360-sudut180)*(sudut360-sudut270));
		if (OutSudut>=360) OutSudut-=360;
	}
	return OutSudut;
}

uint16_t DataSudut(uint16_t Data)
{		
		static uint16_t lastData;
		if(Data<DataMin) 			Data = lastData;
		else if(Data>DataMax)	Data = lastData;
		lastData = Data;
		return ((float)(Data-DataMin)/(float)(DataMax-DataMin)*3599);//359.999
}

short absShort(short x)
{		if(x<0)	return -x;
		else		return x;
}	

//void KontrolPutar(void)
//{
// char dString[20];
//	long u;
//	short Error, dError, buffer; 
//	static short lastError=0, iError=0;
//  short sudutMagnetSekarang; 
//	static short sudutMagnetAkhir, flagSimpanSudut=0, TergetSudutTerakhir=0; 
//	
//	sudutMagnetSekarang = (int) DataSudut(Difference);//((float)AMS_5600_BacaSudut()*0.87976539589442815249266862170088F);
//	Sudut = sudutMagnetSekarang;

////	sprintf(dString,"%d\n", (int)DataSudut(Difference)); 
////	HAL_UART_Transmit(&huart2, dString, strlen(dString), 20);
//	
//	
//	if (sudutMagnetSekarang>3600) sudutMagnetSekarang-=3600;
//	if (flagSimpanSudut==0) { sudutMagnetAkhir = sudutMagnetSekarang; flagSimpanSudut=1;}
//	
//	// menghitung putaran magnet
//	buffer = sudutMagnetAkhir-sudutMagnetSekarang; // untuk menghitung selisih sudut magnet
//	if (buffer>1800||buffer<-1800) posisi*=-1; //melewati 1 putaran	
//	sudutMagnetAkhir = sudutMagnetSekarang;
//	buffer = sudutMagnetSekarang-(posisi*3600);
//	if(buffer>=3600) buffer-=3600;
//	sudutRoda = buffer;
//	// menghitung Error Sudut Roda
//	Error = sudutRoda-TargetSudut;
//	//Error = TargetSudut-sudutRoda;
//	// mengkoreksi kondisi melewati 1 putaran penuh
//	if(Error>3600) Error-=7200;
//	else if(Error<-	3600) Error+=7200;
//	// memberi batas nilai Error maksimal
//	if(Error>LimitErrorSudut) Error=LimitErrorSudut;
//	else if(Error<-LimitErrorSudut) Error=-LimitErrorSudut;
//	
//	// menghitung Integral Error Sudut Roda yang diberi limit
//	if(Error>100) 			buffer = 100;
//	else if(Error<-100) buffer = -100;
//	else buffer = Error;
//	iError+=(float)buffer*0.2;				// pengaturan respon integral Error
//	// memberi batas nilai Integral Error maksimal
//	if(iError>1000) iError-=2500;
//	else if(iError<-1000) iError+=2500;
//	
//	if(TergetSudutTerakhir!=TargetSudut) dError=0;
//	else
//	{	// menghitung Differensial Error Sudut Roda
//		dError = Error-lastError;
//		// mengkoreksi kondisi melewati 1 putaran penuh
//		if(dError>3600) dError-=7200;
//		else if(dError<-3600) dError+=7200;
//	}
//	TergetSudutTerakhir = TargetSudut;
//	// menyimpan nilai Error Terakhir
//	lastError = Error;
//	
//	// PID Control
//	if(absShort(Error)<KP)
//	{	u = (long)Error* (long)(KP+KP-absShort(Error));
//	}	
//	else
//	{	u = (long)Error* (long)KP;	
//	}
//	u+=(long)dError*(long)KD;
//	
//	// memberi batas terhadap nilai kontrol terhadap nilai PWM maksimal
//	if(u>50000) u = 50000;
//	else if(u<-50000) u = -50000;
//	// menghitung nilai PWM yang harus dikirimkan ke motor
//	PWM = u/200;	
//	// kirim nilai PWM baru pada proses komutasi motor
//	if(flagOverTemperature==0) Komutasi();
//////	sprintf(dString,"%d %d %d\n", TargetSudut, sudutMagnetSekarang, PWM); 
//////	HAL_UART_Transmit(&huart2, dString, strlen(dString), 20);
//}	
short Buff;
short SudutKalib;
//Fungsi Kontrol Putar 1:1
void KontrolPutar(void)
{
	char dString[20];
	long u;
	short Error, dError, buffer; 
	static short lastError=0, iError=0;
  short sudutMagnetSekarang; 
	static short sudutMagnetAkhir, flagSimpanSudut=0, TergetSudutTerakhir=0; 
	
	SudutKalib = (short)((ProgramKablirasiSudutSwerve((int) DataSudut(Difference))*10));
	
	
	//sudutMagnetSekarang = (int) DataSudut(Difference);//((float)AMS_5600_BacaSudut()*0.87976539589442815249266862170088F);
	
	sudutMagnetSekarang = SudutKalib;
	Sudut = sudutMagnetSekarang;
	
	KirimSudut = (int) DataSudut(Difference);
//	sprintf(dString,"%d\n", (int)DataSudut(Difference)); 
//	HAL_UART_Transmit(&huart2, dString, strlen(dString), 20);
	
	
	//if (sudutMagnetSekarang>3600) sudutMagnetSekarang-=3600;
	//if (flagSimpanSudut==0) { sudutMagnetAkhir = sudutMagnetSekarang; flagSimpanSudut=1;}
	
	// menghitung putaran magnet
	//buffer = sudutMagnetAkhir-sudutMagnetSekarang; // untuk menghitung selisih sudut magnet
	//if (buffer>1800||buffer<-1800) posisi*=-1; //melewati 1 putaran	
	///sudutMagnetAkhir = sudutMagnetSekarang;
	//buffer = sudutMagnetSekarang-(posisi*3600);
	//if(buffer>=3600) buffer-=3600;
	
	sudutRoda = sudutMagnetSekarang;
	// menghitung Error Sudut Roda
	
	//Error = sudutRoda-TargetSudut;
	
		short target = TargetSudut;
	#if (RodaId == Slave4)
			target = 3600 - target;          // mirror sumbu X khusus roda 2
			if (target >= 3600) target -= 3600;
	#endif	
	
	Error =  sudutRoda-target ;	
	
	//Error = TargetSudut-sudutRoda;	//BLDC
	// mengkoreksi kondisi melewati 1 putaran penuh
	if(Error>1800) Error-=3600;
	else if(Error<-1800) Error+=3600;
	Buff = Error;
	// memberi batas nilai Error maksimal
	if(Error>LimitErrorSudut) Error=LimitErrorSudut;
	else if(Error<-LimitErrorSudut) Error=-LimitErrorSudut;
	
//	// menghitung Integral Error Sudut Roda yang diberi limit
//	if(Error>100) 			buffer = 100;
//	else if(Error<-100) buffer = -100;
//	else buffer = Error;
//	iError+=(float)buffer*0.2;				// pengaturan respon integral Error
//	// memberi batas nilai Integral Error maksimal
//	if(iError>1000) iError-=2500;
//	else if(iError<-1000) iError+=2500;
	
	//Menghitung Integral Respon 
	if(Error>LimitErrorSudut) buffer=LimitErrorSudut;
	else if(Error<-LimitErrorSudut) buffer=-LimitErrorSudut;
	
	iError+=(float)buffer*KI;//Respon Inetegral
	//Membatasi Nilai iError
//	if(iError>500) iError-=2500;
//	else if(iError<-500) iError+=2500;
//	
	if(iError>1000) iError=1000;
	else if(iError<-1000) iError=1000;
	
	//Pengaturan Respon Derivatif
	if(TergetSudutTerakhir!=TargetSudut) dError=0;
	else
	{	// menghitung Differensial Error Sudut Roda
		dError = Error-lastError;
		// mengkoreksi kondisi melewati 1 putaran penuh
		if(dError>1800) dError-=3600;
		else if(dError<-1800) dError+=3600;
	}
	
	TergetSudutTerakhir = TargetSudut;
	// menyimpan nilai Error Terakhir
	lastError = Error;
	
	// PID Control
	if(absShort(Error)<KP)
	{	u = (long)Error* (long)(KP+KP-absShort(Error));
	}	
	else
	{	u = (long)Error*(long)KP;	
	}
	//u+=(long)dError*(long)KD;
	u+=((long)dError*(long)KD);
	
	// memberi batas terhadap nilai kontrol terhadap nilai PWM maksimal
	if(u>50000) u = 50000;
	else if(u<-50000) u = -50000;
	// menghitung nilai PWM yang harus dikirimkan ke motor
//	PWM = u/200;	
//	
//	//PersamaanPWM = 
//	// kirim nilai PWM baru pada proses komutasi motor
//	if(flagOverTemperature==0) Komutasi();
	//Motor DC
	PWM = u/5;	
	
	//PersamaanPWM = 
	// kirim nilai PWM baru pada proses komutasi motor
	if(flagOverTemperature==0) Motor(PWM);
//	sprintf(dString,"%d %d %d\n", TargetSudut, sudutMagnetSekarang, PWM); 
//	HAL_UART_Transmit(&huart2, dString, strlen(dString), 20);
	
//	// 10 - 20
//	BuffPWM = u/200;
//	if(BuffPWM>=0)
//	{
//		PersamaanPWM = ((230*BuffPWM)+2500)/240;
//		PWM=PersamaanPWM;
//		if(flagOverTemperature==0) Komutasi();
//	}
//	else 
//	{
//		PersamaanPWM = ((-230*BuffPWM)+2500)/-240;
//		PWM=PersamaanPWM;
//		if(flagOverTemperature==0) Komutasi();
//	}
//	
//	// 5 - 25
//	BuffPWM = u/200;
//	if(BuffPWM>=0)
//	{
//		PersamaanPWM = ((225*BuffPWM)+5000)/245;
//		PWM=PersamaanPWM;
//		if(flagOverTemperature==0) Komutasi();
//	}
//	else 
//	{
//		PersamaanPWM = ((-225*BuffPWM)+5000)/-245;
//		PWM=PersamaanPWM;
//		if(flagOverTemperature==0) Komutasi();
//	}
}	
////Fungsi Kontrol Putar 1:1
//void KontrolPutar(void)
//{
//	char dString[20];
//	long u;
//	short Error, dError, iError; 
//	static short lastError=0;
//	short sudutMagnetSekarang; 
//	static short sudutMagnetAkhir, flagSimpanSudut=0, TergetSudutTerakhir=0; 
//	
//	sudutMagnetSekarang = (int) DataSudut(Difference);//((float)AMS_5600_BacaSudut()*0.87976539589442815249266862170088F);
//	Sudut = sudutMagnetSekarang;
//	
//	KirimSudut = Sudut;
////	sprintf(dString,"%d\n", (int)DataSudut(Difference)); 
////	HAL_UART_Transmit(&huart2, dString, strlen(dString), 20);
//	
//	
//	//if (sudutMagnetSekarang>3600) sudutMagnetSekarang-=3600;
//	//if (flagSimpanSudut==0) { sudutMagnetAkhir = sudutMagnetSekarang; flagSimpanSudut=1;}
//	
//	// menghitung putaran magnet
//	//buffer = sudutMagnetAkhir-sudutMagnetSekarang; // untuk menghitung selisih sudut magnet
//	//if (buffer>1800||buffer<-1800) posisi*=-1; //melewati 1 putaran	
//	///sudutMagnetAkhir = sudutMagnetSekarang;
//	//buffer = sudutMagnetSekarang-(posisi*3600);
//	//if(buffer>=3600) buffer-=3600;
//	
//	sudutRoda = sudutMagnetSekarang;
//	// menghitung Error Sudut Roda
//	//Error = sudutRoda-TargetSudut;
//	Error = TargetSudut-sudutRoda;
//	// mengkoreksi kondisi melewati 1 putaran penuh
//	if(Error>1800) Error-=3600;
//	else if(Error<-	1800) Error+=3600;
//	// memberi batas nilai Error maksimal
//	if(Error>LimitErrorSudut) Error=LimitErrorSudut;
//	else if(Error<-LimitErrorSudut) Error=-LimitErrorSudut;
//	
//	//Derivatif 
//	dError = Error - lastError;
//	if(dError>1800)dError-=3600;
//	else if(dError<-1800)dError+=3600;
//	
//	//Integral
//	 
//	iError = Error + lastError;
//	if(iError>1800)iError-=3600;
//	else if(iError<-1800)iError+=3600;
//	
//	
//	u=(short)(KP*Error)+(short)(KI*iError)+(short)(KD*dError);
//	
//	//memberi batas terhadap nilai kontrol terhadap nilai PWM maksimal
//	if(u>50000) u = 50000;
//	else if(u<-50000) u = -50000;
//	// menghitung nilai PWM yang harus dikirimkan ke motor
//	PWM = u/200;	
//	// kirim nilai PWM baru pada proses komutasi motor
//	if(flagOverTemperature==0) Komutasi();
//	
//	//
//	
//}	

//Fungsi Komutasi BLDC Motor 5010
void  Komutasi(void)
{ static signed short lastPWM=0;
	short PWMplus, PWMminus;
	unsigned char NilaiHallSensorSekarang = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==GPIO_PIN_SET) 
	{	NilaiHallSensorSekarang|=1;//1
		SetLedHallA;
	}
	else		
	{	ClrLedHallA;
	}
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==GPIO_PIN_RESET) 
	{	NilaiHallSensorSekarang|=4;//4
		SetLedHallB;
	}
	else		
	{	ClrLedHallB;
	}	
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET) 
	{	NilaiHallSensorSekarang|=2;//2
		SetLedHallC;
	}
	else		
	{	ClrLedHallC;
	}
		
	if( PWM==0 || (lastPWM*PWM)<0)	// jika pwm kurang dari PWM minimum atau setelah berbalik arah
	{		TIM1->CCR1 = 0;						SetB(12);	//netral
			TIM1->CCR2 = 0;						SetB(14);	//plus
			TIM1->CCR3 = 0;						SetB(15); //minus
	}	
	else 
	{			PWMplus = CenterPWM+PWM;
				PWMminus = CenterPWM-PWM;
				if(NilaiHallSensorSekarang== 6)			//110
				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
						TIM1->CCR3 = PWMminus;			SetB(15); //minus
				}
				else if(NilaiHallSensorSekarang==2)	//0B010) ---
				{		TIM1->CCR3 = CenterPWM;			ClrB(15); //netral
						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
				}
				else if(NilaiHallSensorSekarang==3)	//0B011) ---
				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
				}
				else if(NilaiHallSensorSekarang==1)	//0B001)
				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
				}
				else if(NilaiHallSensorSekarang==5)	//0B101)
				{		TIM1->CCR3 = 250;						ClrB(15);	//netral
						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
				}
				else if(NilaiHallSensorSekarang==4)	//0B100)
				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
						TIM1->CCR3 = PWMminus;			SetB(15);	//plus
				}
				else
				{		ClrB(12);	//netral
						ClrB(14);	//netral
						ClrB(15);	//netral
				}
	}
	lastPWM = PWM;
}	

void Motor(int Speed){
	if(Speed>=0)
	{
 
		SetLedHallA; ClrLedHallB;
		if(Speed>=5000)Speed=5000;
		TIM1->CCR3 = Speed;	SetB(15);
		TIM1->CCR1 = 0;	SetB(12);
	}
	else 
	{
		SetLedHallB; ClrLedHallA;
		if(Speed<=-5000)Speed=-5000;
		TIM1->CCR3 = 0;	SetB(15);
		TIM1->CCR1 = -Speed;	SetB(12);
	}
}
////Fungsi Komutasi BLDC Motor 5065
//void  Komutasi(void)
//{ static signed short lastPWM=0;
//	short PWMplus, PWMminus;
//	unsigned char NilaiHallSensorSekarang = 0;
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==GPIO_PIN_SET) 
//	{	NilaiHallSensorSekarang|=1;//1
//		SetLedHallA;
//	}
//	else		
//	{	ClrLedHallA;
//	}
//	
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==GPIO_PIN_RESET) 
//	{	NilaiHallSensorSekarang|=4;//4
//		SetLedHallB;
//	}
//	else		
//	{	ClrLedHallB;
//	}	
//	
//	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET) 
//	{	NilaiHallSensorSekarang|=2;//2
//		SetLedHallC;
//	}
//	else		
//	{	ClrLedHallC;
//	}
//		
//	if( PWM==0 || (lastPWM*PWM)<0)	// jika pwm kurang dari PWM minimum atau setelah berbalik arah
//	{		TIM1->CCR1 = 0;						SetB(12);	//netral
//			TIM1->CCR2 = 0;						SetB(14);	//plus
//			TIM1->CCR3 = 0;						SetB(15); //minus
//	}	
//	else 
//	{	
//			if(PWM>=0)
//			{
//				PWMplus = CenterPWM+PWM;
//				PWMminus = CenterPWM-PWM;
//				if(NilaiHallSensorSekarang== 4)			//110
//				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//						TIM1->CCR3 = PWMminus;			SetB(15); //minus
//				}
//				else if(NilaiHallSensorSekarang==6)	//0B010) ---
//				{		TIM1->CCR3 = CenterPWM;			ClrB(15); //netral
//						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//				}
//				else if(NilaiHallSensorSekarang==2)	//0B011) ---
//				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//				}
//				else if(NilaiHallSensorSekarang==3)	//0B001)
//				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//				}
//				else if(NilaiHallSensorSekarang==1)	//0B101)
//				{		TIM1->CCR3 = 250;						ClrB(15);	//netral
//						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//				}
//				else if(NilaiHallSensorSekarang==5)	//0B100)
//				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//						TIM1->CCR3 = PWMminus;			SetB(15);	//plus
//				}
//				else
//				{		ClrB(12);	//netral
//						ClrB(14);	//netral
//						ClrB(15);	//netral
//				}
//			}
//			else
//			{
//				PWMplus = CenterPWM+PWM;
//				PWMminus = CenterPWM-PWM;
//				if(NilaiHallSensorSekarang== 6)			//110
//				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//						TIM1->CCR3 = PWMminus;			SetB(15); //minus
//				}
//				else if(NilaiHallSensorSekarang==2)	//0B010) ---
//				{		TIM1->CCR3 = CenterPWM;			ClrB(15); //netral
//						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//						TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//				}
//				else if(NilaiHallSensorSekarang==3)	//0B011) ---
//				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//						TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//				}
//				else if(NilaiHallSensorSekarang==1)	//0B001)
//				{		TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//						TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//				}
//				else if(NilaiHallSensorSekarang==5)	//0B101)
//				{		TIM1->CCR3 = 250;						ClrB(15);	//netral
//						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//						TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//				}
//				else if(NilaiHallSensorSekarang==4)	//0B100)
//				{		TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//						TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//						TIM1->CCR3 = PWMminus;			SetB(15);	//plus
//				}
//				else
//				{		ClrB(12);	//netral
//						ClrB(14);	//netral
//						ClrB(15);	//netral
//				}
//			}
//	}
//	lastPWM = PWM;
//}	
unsigned long HallSensorInterval; 

//Timer Input Capture Callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	//Timer Komutasi
//		if(htim->Instance == TIM3)
//		{		//HallSensorInterval = TIM3->CNT;
//				//TIM3->CNT=0;
//				ClrB(12);	//netral
//				ClrB(14);	//netral
//				ClrB(15);	//netral
//				if(flagOverTemperature==0) Komutasi();
//		}	
		
	//Start AS5600 PWM Output
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (Is_First_Captured == 0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			Is_First_Captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured == 1)
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
				//flagSensor = 1;
			}
			/*
				else if(IC_Val1>IC_Val2)
				{
				Difference = (0xffff -IC_Val1)+IC_Val2;
				}

				float RefClock = TIMCLOCK/(PRESCALAR);

				float mFactor = 1000000/RefClock;

				Duty = Difference*mFactor;

				Frequency = RefClock/Difference;

				__HAL_TIM_SET_COUNTER(htim,0);
			*/
			Is_First_Captured = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

		}
	}
}

//Timer Period CallBack 10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		static uint32_t FilterADC[5]={0,0,0,0,0};	// untuk moving window average
		static uint32_t	Total=0;
		static int FilterCounter=0, counterError=0;
		float temperature;
		KontrolPutar();

//		if(htim->Instance == TIM4)
//		{		
//			Total += adcTemp;											// data yang baru simasukkan
//			Total -= FilterADC[FilterCounter];		// data yanng lama dikurangkan
//			FilterADC[FilterCounter] = adcTemp;		// data pada index di uppdate
//			if(++FilterCounter>=5)FilterCounter=0;
//			temperature = ((1.43-((float)(Total/5)*3.3/4096))/0.0043) + 25;
////			char xx[10];
////			sprintf(xx,"%d %.1f\n",Total/5, temperature);
////			HAL_UART_Transmit(&huart3,xx,sizeof(xx),100);
//			if(temperature>70)	flagOverTemperature=1;
//			
//			if(flagOverTemperature==0)
//			{		
//				KontrolPutar();
//				//Motor(-2000);
//			}
//			else		// jika terjadi kondisi over temperature
//			{		ClrB(15);	//netral
//					//ClrB(12);	//netral
//					ClrB(14);	//netral	
//					if(temperature<70)	{ flagOverTemperature=0;counterError=0;Komutasi();}
//					else
//					{	if(counterError==0)	
//						{	SetLedHallA;
//							//SetLedHallB;
//							SetLedHallC;
//						}
//						else if(counterError==25)
//						{	ClrLedHallA;
//							//ClrLedHallB;
//							ClrLedHallC;
//						}	
//					if(++counterError>=50)counterError=0;
//					}
//			}	
//		}	
}	

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
