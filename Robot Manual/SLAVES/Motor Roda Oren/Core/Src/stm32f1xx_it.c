/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "../Inc/myDEF.h"
#include "stdio.h"
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern	ADC_HandleTypeDef hadc1; 
extern 	volatile unsigned char NilaiHallSensor;
extern 	signed short PWM;
extern void  	Komutasi(void);
//extern 	UART_HandleTypeDef huart3;
extern	volatile uint8_t	flagOverTemperature;
extern uint32_t adcTemp;


//Serial
void HitungDataUSART3(uint8_t c);
uint8_t volatile RxData[1];
uint8_t DataRX;
short SudutFromSerial=0;


//Counter
uint8_t Trig=0;
int CounterHall=0;
float RPM;
float LastRPM;
int Var=0;
//Rising
static int MemBit[8];
uint8_t LogicHall1=0;
uint8_t LogicHall2=0;
uint8_t LogicHall3=0;
uint8_t LogicHall4=0;
uint8_t LogicHall5=0;
uint8_t LogicHall6=0;

#define PI 3.14159265359f
#define Radius 0.07 //satuan m

float MPerS=0;

//Speed Kontrol
//RPM = 0-1200
float ActualSpeed=0;
float SetPointSpeed=0;
#define LimitError 20.0
float Temp;

float Error=0,IError=0,LastErrorI=0, DError=0,LastDError=0;
float Kontrol=0,PID=0;
float u=0;
float Kp=0,Ki=0,Kd=0;

float MaxKec, MinKec;
float SendToPWM = 0.283; // PWM/MaxActualSpeed 250/1200
float MaxPower = 1.2;
float MinPower = 0.75;
float PIDMax = 100;

short Inc=0;

//char Buff[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
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
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
//	HAL_UART_Receive_IT(&huart2,(uint8_t*)RxData,1);
//	HitungDataUSART3(RxData[0]); 
  /* USER CODE END USART2_IRQn 1 */
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
	HAL_UART_Receive_IT(&huart3,(uint8_t*)RxData,1);
	HitungDataUSART3(RxData[0]); 
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HitungDataUSART3(uint8_t c)
{ 
	static uint8_t Flag=0,Counter=0,Data[3];
	if(Flag==0)
	{
		if(c==0x80)
		{
			Flag=1;
			//Counter=0;
		}
		else Flag=0;
	}
	else if(Flag==1)
	{
		if(c==0xFF)
		{
			Flag=2;
			Counter=0;
		}
		else Flag = 0;
		
	}
	else if (Flag==2)
	{
		Data[Counter] = c;
		Counter++;
		if(Counter==3)
		{
			if((Data[1]== (Data[0]^0xFF))&&Data[2]==Data[1])
			{
				if(flagOverTemperature==0)
				{			
          //PWM = (signed char) (Data[0]/1.33); //motor 1
					//PWM = (signed char) (Data[0]/1.05); //motor 3
					PWM = (signed char) Data[0];
					//PWM*=-1;
					//PWM/=2;
					//Komutasi();
				}
				Flag=0;
			}
			else Flag = 0;
		}
		
	}
}

//void HitungDataUSART3(uint8_t c)
//{
//	static uint8_t Flag=0,Counter=0,Data[2];
//	if(Flag==0)
//	{
//		if(c==0x80)
//		{
//			Flag=1;
//			Counter=0;
//		}
//		else Flag=0;
//	}
//	else if (Flag==1)
//	{
//		Data[Counter] = c;
//		Counter++;
//		if(Counter==1)
//		{
//				if(flagOverTemperature==0)
//				{			
//					PWM = (signed char) Data[0];
//					Komutasi();
//				}
//				Flag=0;
//				//else Flag = 0;
//		}
//		
//	}
//}



#define CenterPWM 127 //PWM dengan range -127 s/d 127
void Komutasi()
{
	static signed short lastPWM=0;
	short PWMplus, PWMminus;
	unsigned char NilaiHallSensorSekarang = 0;
	if (flagOverTemperature==0)
	{	
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==GPIO_PIN_SET) 
		{	
			NilaiHallSensorSekarang|=1;
			SetLedHallA;
			LogicHall1=1;
			
		}
		else		
		{	
			ClrLedHallA;
			LogicHall1=0;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==GPIO_PIN_SET) //Jika BLDC Motor Putar Dibalik logikanya
		{	
			NilaiHallSensorSekarang|=2;
			SetLedHallB;
			LogicHall2=1;
		}
		else		
		{	
			LogicHall2=0;
			ClrLedHallB;
		}	
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET) 
		{	
			NilaiHallSensorSekarang|=4;
			SetLedHallC;
			LogicHall3=1;
		}
		else		
		{	
			LogicHall3=0;
			ClrLedHallC;
		}
		
		
//		//Rising
//		if(RisingEdge(LogicHall1,0))
//		{
//			CounterHall+=1;
//		}
//		if(RisingEdge(LogicHall2,1))
//		{
//			CounterHall+=1;
//		}	
//		if(RisingEdge(LogicHall3,2))
//		{
//			CounterHall+=1;
//		}
		
		if(PWM == 0 || (lastPWM*PWM<0))
		{
			TIM1->CCR1 = 0;		SetB(12); //A
			TIM1->CCR2 = 0;		SetB(14); //A
			TIM1->CCR3 = 0;		SetB(15); //A
						
		}
		
		else
		{
			PWMplus = CenterPWM+PWM;
			PWMminus = CenterPWM-PWM;
			
			if(NilaiHallSensorSekarang== 6)			//110 6
			{		
				TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
				TIM1->CCR2 = PWMplus;				SetB(14);	//plus
				TIM1->CCR3 = PWMminus;			SetB(15); //minus
				
				//Speed Ketika 1 Pole
				if(Trig==0){
					CounterHall++;
					Trig=1;
				}
			}
			else if(NilaiHallSensorSekarang==2)	//0B010) 2
			{		
				Trig=0;
				TIM1->CCR3 = CenterPWM;			ClrB(15); //netral
				TIM1->CCR1 = PWMminus;			SetB(12);	//minus
				TIM1->CCR2 = PWMplus;				SetB(14);	//plus
			}
			else if(NilaiHallSensorSekarang==3)	//0B011) 3
			{		
				Trig=0;
				TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
				TIM1->CCR1 = PWMminus;			SetB(12);	//minus
				TIM1->CCR3 = PWMplus;				SetB(15);	//plus
			}
			else if(NilaiHallSensorSekarang==1)	//0B001) 1
			{		
				Trig=0;
				TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
				TIM1->CCR2 = PWMminus;			SetB(14);	//minus
				TIM1->CCR3 = PWMplus;				SetB(15);	//plus
			}
			else if(NilaiHallSensorSekarang==5)	//0B101) 5
			{		
				Trig=0;
				TIM1->CCR3 = CenterPWM;			ClrB(15);	//netral
				TIM1->CCR1 = PWMplus;				SetB(12);	//plus
				TIM1->CCR2 = PWMminus;			SetB(14);	//minus
			}
				else if(NilaiHallSensorSekarang==4)	//0B100)4
			{		
				Trig=0;
				TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
				TIM1->CCR1 = PWMplus;				SetB(12);	//plus
				TIM1->CCR3 = PWMminus;			SetB(15);	//plus
			}
			else
			{		
				ClrB(12);	//netral
				ClrB(14);	//netral
				ClrB(15);	//netral
			}	
		}
	}
	else
	{
		ClrB(12);
		ClrB(14);
		ClrB(15);
		
	}
	NilaiHallSensor = NilaiHallSensorSekarang;
}



//hitam
//#define CenterPWM 127 //PWM dengan range -127 s/d 127
//void Komutasi()
//{
//	static signed short lastPWM=0;
//	short PWMplus, PWMminus;
//	unsigned char NilaiHallSensorSekarang = 0;
//	if (flagOverTemperature==0)
//	{	
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==GPIO_PIN_SET) 
//		{	
//			NilaiHallSensorSekarang|=1;//441122
//			SetLedHallA;
//			LogicHall1=1;
//			
//		}
//		else		
//		{	
//			ClrLedHallA;
//			LogicHall1=0;
//		}
//		
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==GPIO_PIN_RESET) //Jika BLDC Motor Putar Dibalik logikanya
//		{	
//			NilaiHallSensorSekarang|=2;//214214
//			SetLedHallB;
//			LogicHall2=1;
//		}
//		else		
//		{	
//			LogicHall2=0;
//			ClrLedHallB;
//		}	
//		
//		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET) 
//		{	
//			NilaiHallSensorSekarang|=4;//122441
//			SetLedHallC;
//			LogicHall3=1;
//		}
//		else		
//		{	
//			LogicHall3=0;
//			ClrLedHallC;
//		}
//		
//		
////		//Rising
////		if(RisingEdge(LogicHall1,0))
////		{
////			CounterHall+=1;
////		}
////		if(RisingEdge(LogicHall2,1))
////		{
////			CounterHall+=1;
////		}	
////		if(RisingEdge(LogicHall3,2))
////		{
////			CounterHall+=1;
////		}
//		
//		if(PWM == 0 || (lastPWM*PWM<0))
//		{
//			TIM1->CCR1 = 0;		SetB(12); //A
//			TIM1->CCR2 = 0;		SetB(14); //A
//			TIM1->CCR3 = 0;		SetB(15); //A
//						
//		}
//		
//		else
//		{
//			PWMplus = CenterPWM+PWM;
//			PWMminus = CenterPWM-PWM;
//			
//			if(NilaiHallSensorSekarang== 6)			//110 6
//			{		
//				TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//				TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//				TIM1->CCR3 = PWMminus;			SetB(15); //minus
//				
//				//Speed Ketika 1 Pole
//				if(Trig==0){
//					CounterHall++;
//					Trig=1;
//				}
//			}
//			else if(NilaiHallSensorSekarang==2)	//0B010) 2
//			{		
//				Trig=0;
//				TIM1->CCR3 = CenterPWM;			ClrB(15); //netral
//				TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//				TIM1->CCR2 = PWMplus;				SetB(14);	//plus
//			}
//			else if(NilaiHallSensorSekarang==3)	//0B011) 3
//			{		
//				Trig=0;
//				TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//				TIM1->CCR1 = PWMminus;			SetB(12);	//minus
//				TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//			}
//			else if(NilaiHallSensorSekarang==1)	//0B001) 1
//			{		
//				Trig=0;
//				TIM1->CCR1 = CenterPWM;			ClrB(12);	//netral
//				TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//				TIM1->CCR3 = PWMplus;				SetB(15);	//plus
//			}
//			else if(NilaiHallSensorSekarang==5)	//0B101) 5
//			{		
//				Trig=0;
//				TIM1->CCR3 = CenterPWM;			ClrB(15);	//netral
//				TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//				TIM1->CCR2 = PWMminus;			SetB(14);	//minus
//			}
//				else if(NilaiHallSensorSekarang==4)	//0B100)4
//			{		
//				Trig=0;
//				TIM1->CCR2 = CenterPWM;			ClrB(14);	//netral
//				TIM1->CCR1 = PWMplus;				SetB(12);	//plus
//				TIM1->CCR3 = PWMminus;			SetB(15);	//plus
//			}
//			else
//			{		
//				ClrB(12);	//netral
//				ClrB(14);	//netral
//				ClrB(15);	//netral
//			}	
//		}
//	}
//	else
//	{
//		ClrB(12);
//		ClrB(14);
//		ClrB(15);
//		
//	}
//}

void SpeedKontrol()
{
	
	//0.001 0.015
	//Speed Roda 0-1200 RPM
	//Kp = 0.001; Ki = 0.035; Kd = 0;
	//Kp = 0.001; Ki = 0.035; Kd = 0;
	Kp = 0.0015; Ki = 0.055; Kd = 0;
	
	Error = SetPointSpeed - ActualSpeed;
	
	//Membatasi Error
	if(Error>50)Error=50;
	else if(Error<-50)Error = -50;
	
	DError = Error - LastDError;
	LastDError = Error;
	IError+=Error;
	
	//Membatasi Erorr Integral
	if(IError>150)IError = 150;
	else if (IError<-150)IError = -150;
	
	//Kalkulasi PID
	PID = (Kp*Error)+(Ki*(IError*0.01))+(Kd*(DError/0.01));
	
	
	if(PID>PIDMax) PID = PIDMax;
	else if(PID<-PIDMax) PID =-PIDMax;
	
	Kontrol+=PID;
	Inc+=1;
	
	//Batas Atas & Bawah terhadap SetPoint
	MaxKec = SetPointSpeed*MaxPower;
	MinKec = SetPointSpeed*MinPower;
	
	if(SetPointSpeed>0)
	{
		if(Kontrol>MaxKec)Kontrol = MaxKec;
		else if(Kontrol<MinKec) Kontrol = MinKec;
	}
	else if(SetPointSpeed<0)
	{
		if(Kontrol<MaxKec)Kontrol =MaxKec;
		else if(Kontrol>MinKec) Kontrol =MinKec;
	}
	
	
	u=Kontrol*SendToPWM;
	
	if(u>=250) u = 250;
	else if(u<=-250) u =-250;
	
	if(SetPointSpeed==0)
	{
		PWM = 0;
		Error=0;
		IError=0;
		DError=0;
		PID = 0;
		u=0;
	}
	
	PWM = (short)u;
	if(flagOverTemperature==0)Komutasi();
	
	
	
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)//Feedback Timer untuk HallSensor
	{
		
		Komutasi();
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		static uint32_t FilterADC[5]={0,0,0,0,0};	// untuk moving window average
		static uint32_t	Total=0;
		static int FilterCounter=0, counterError=0;
		float temperature;

		if(htim->Instance == TIM2)
		{
			Komutasi();
//			
//			Total += adcTemp;											// data yang baru simasukkan
//			Total -= FilterADC[FilterCounter];		// data yanng lama dikurangkan
//			FilterADC[FilterCounter] = adcTemp;		// data pada index di uppdate
//			if(++FilterCounter>=5)FilterCounter=0;
//			temperature = ((1.43-((float)(Total/5)*3.3/4096))/0.0043) + 25;
////			char xx[10];
////			sprintf(xx,"%d %.1f\n",Total/5, temperature);
////			HAL_UART_Transmit(&huart3,xx,sizeof(xx),100);
//			if(temperature>200)	flagOverTemperature=1;
//			
//			if(flagOverTemperature==0)
//			{
//				//SpeedKontrol();
//				//Komutasi();
//			}	
//			else		// jika terjadi kondisi over temperature
//			{		ClrB(15);	//netral
//					ClrB(12);	//netral
//					ClrB(14);	//netral	
//					if(temperature<70){ flagOverTemperature=0;counterError=0;Komutasi();}
//					else
//					{	if(counterError==0)	
//						{	SetLedHallA;
//							SetLedHallB;
//							SetLedHallC;
//						}
//						else if(counterError==25)
//						{	ClrLedHallA;
//							ClrLedHallB;
//							ClrLedHallC;
//						}	
//					if(++counterError>=50)counterError=0;
//					}
//			}
		}	
//		if(htim->Instance == TIM4)
//		{
			//(GPIOB,GPIO_PIN_5);
//			Var++;		
//			if(Var>170){
//				RPM = ((CounterHall*120.0)/12.0);//0.0942
//				MPerS = (2*PI*Radius)*(RPM/60);
//				
//				ActualSpeed = (float)RPM;
//				if(SetPointSpeed>=0){
//					
//					ActualSpeed = RPM;
//				}
//				else{
//					ActualSpeed = -RPM;
//				}
//				//LastRPM = RPM;
//				CounterHall=0;
//				Var = 0;
//			}
			
			
//			Var++;		
//			if(Var>16){
//				RPM = ((CounterHall*120.0)/12.0)*10.0;//0.0942
//				MPerS = (2*PI*Radius)*(RPM/60);
//				
//				ActualSpeed = (float)RPM;
//				if(SetPointSpeed>=0){
//					
//					ActualSpeed = RPM;
//				}
//				else{
//					ActualSpeed = -RPM;
//				}
//				//LastRPM = RPM;
//				CounterHall=0;
//				Var = 0;
//			}
//		}
}
/* USER CODE END 1 */
