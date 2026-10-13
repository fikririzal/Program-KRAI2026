/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\Inc\myDEF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void KontrolPutar(void);
volatile unsigned char NilaiHallSensor;
signed short PWM;
short TargetSudut, posisi, offsetSudutRoda=0;
extern void  Komutasi(void);
unsigned char KP=100, KD=120; 

// pemilihan Can ID dari slave pindah ke file myDEF.h

CAN_RxHeaderTypeDef myCanRxHeader;
uint32_t TxMailBox;
uint8_t CanTxData[8];
CAN_FilterTypeDef sCanFilter;	

int AMS_5600_BacaSudut()
{
  uint8_t sudut[2];
	HAL_I2C_Mem_Read(&hi2c1, 0x36<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, sudut, 2, 200);
  return sudut[0]*256+sudut[1]; 
}

void SetSudutRoda(short sudut)	// 0 sampai 7200
{	
	sudut+=offsetSudutRoda;
	if(sudut>7200) 						sudut -=7200;
	else if ((sudut<-7200)) 	sudut +=7200;
	TargetSudut = sudut;
}	

void CariTitikNol()
{		
		int i;
		PWM = 125;	Komutasi();
		for(i=0;i<10;i++)	// memastikan posisi magnet terbaca sebanyak 10 x
		{	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==GPIO_PIN_SET);
			HAL_Delay(1);
		}
		posisi = 1;
		PWM = 0;		Komutasi(); 
		HAL_Delay(100);
}	

//program ini bisa dihapus untuk motor slave
CAN_TxHeaderTypeDef myCanTxHeader;

// memastikan bahwa nilai pwm roda yang dikirimkan pada range -127 sampai dengan 127
uint8_t	SetBatasPwmRoda(short PwmRoda)
{	if(PwmRoda>127)PwmRoda=127;
	else if(PwmRoda<-127)PwmRoda=-127;
	return (uint8_t) PwmRoda;
}

int SetKecepatanDanSudutRoda(signed char speed, float heading, uint32_t CanSlaveAddress)
{	uint8_t cksum, headingH, headingL;
	short headingS ;
	if( speed==-128) return 0;	// data kecepatan salah
	heading*=20.0f;
	headingS = heading;
	headingH = headingS>>8;
	headingL = headingS&255;
	cksum = (uint8_t)speed ^ headingH ^ headingL;
	myCanTxHeader.StdId = CanSlaveAddress;
	CanTxData[0] = speed; 
	CanTxData[1] = headingH;
	CanTxData[2] = headingL;
	CanTxData[3] = cksum; 
	HAL_CAN_AddTxMessage(&hcan,&myCanTxHeader,CanTxData,&TxMailBox);
	return 1;
}	

void SetOffsetSudutRoda(float OffsetSudut, uint32_t CanSlaveAddress)
{	uint8_t cksum, header, sudutH, sudutL;
	short sudut;
	OffsetSudut*=20.0f;
	sudut = (short) OffsetSudut;
	if(sudut>3600)sudut-=7200;
	else if(sudut<-3600)sudut+=7200;
	
	header = 0x80;	//-128
	sudutH = sudut>>8;
	sudutL = sudut&255;
	cksum = header ^ sudutH ^ sudutL;
	myCanTxHeader.StdId = CanSlaveAddress;	
	CanTxData[0] = 0x80; 
	CanTxData[1] = sudutH;
	CanTxData[2] = sudutL;
	CanTxData[3] = cksum; 
	HAL_CAN_AddTxMessage(&hcan,&myCanTxHeader,CanTxData,&TxMailBox);
}

void SetKpKd(uint8_t Kp, uint8_t Kd, uint32_t CanSlaveAddress)
{	uint8_t cksum = 1 ^ Kp ^ Kd;
	myCanTxHeader.StdId = CanSlaveAddress;	
	CanTxData[0] = 1; 
	CanTxData[1] = Kp;
	CanTxData[2] = Kd;
	CanTxData[3] = ~cksum; 
	HAL_CAN_AddTxMessage(&hcan,&myCanTxHeader,CanTxData,&TxMailBox);
}	

void RequestSudut(uint32_t CanSlaveAddress)
{	uint8_t cksum = 2 ^ 3 ^ 4;
	myCanTxHeader.StdId = CanSlaveAddress;	
	CanTxData[0] = 2; 
	CanTxData[1] = 3;
	CanTxData[2] = 4;
	CanTxData[3] = ~cksum; 
	HAL_CAN_AddTxMessage(&hcan,&myCanTxHeader,CanTxData,&TxMailBox);
}

void MasterInitStruct(void)
{
	myCanTxHeader.DLC = 4;
	myCanTxHeader.IDE = CAN_ID_STD;
	myCanTxHeader.RTR = CAN_RTR_DATA;
}

// sampai di sini yang bisa si hapus


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i=0; 
	//char dString[10];
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	offsetSudutRoda=0;
	
	myCanRxHeader.DLC = 4;
	myCanRxHeader.IDE = CAN_ID_STD;
	myCanRxHeader.RTR = CAN_RTR_DATA;
	
	sCanFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sCanFilter.FilterIdHigh = RodaCanId<<5;
	sCanFilter.FilterIdLow = 0;
	sCanFilter.FilterMaskIdHigh = 0xFFE0;
	sCanFilter.FilterMaskIdLow = 0;
	sCanFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	sCanFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	sCanFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &sCanFilter);
	
	MasterInitStruct();
	// start CAN Bus
	HAL_CAN_Start(&hcan);
	//HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
 
	for(i=0;i<5;i++)
		{	SetLedHallA;
			SetLedHallB;
			SetLedHallC;
			HAL_Delay(50);
			ClrLedHallA;
			ClrLedHallB;
			ClrLedHallC;
			HAL_Delay(50);
		}
	
		
	// start PWM motor	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			
	// start Hall Sensor	
	HAL_TIMEx_HallSensor_Start_IT(&htim3);	
	// Cari posisi sudut awal (berhenti pada posisi magnet)
	CariTitikNol();	
//	sprintf(&dString[0],"Serial\n");
//	HAL_UART_Transmit(&huart3, dString, strlen(dString), 1000);
	
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {		//if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)!=0)SetLedHallC;
			//else								ClrLedHallC;
//			for(i=0;i<720;i++)
//			{	SetSudutRoda(i*10);
//				HAL_Delay(10);
//			}
//			HAL_Delay(1000);
//			for(i=90;i>0;i--)
//			{	SetSudutRoda(i*80);
//				HAL_Delay(10);
//			}
//			HAL_Delay(1000);
			
// coba master kirim data
//			SetKecepatanDanSudutRoda(0, 0, CanIdRoda1);
//			SetKecepatanDanSudutRoda(0, 0, CanIdRoda2);
//			SetKecepatanDanSudutRoda(0, 0, CanIdRoda3);
//			HAL_Delay(1000);
//			SetKecepatanDanSudutRoda(-60, -90, CanIdRoda1);
//			SetKecepatanDanSudutRoda(-30, 90, CanIdRoda2);
//			SetKecepatanDanSudutRoda(-90, 45, CanIdRoda3);
//			HAL_Delay(1000);
//			SetKecepatanDanSudutRoda(0, -180, CanIdRoda1);
//			SetKecepatanDanSudutRoda(0, 180, CanIdRoda2);
//			SetKecepatanDanSudutRoda(0, 90, CanIdRoda3);
//			HAL_Delay(1000);
//			SetKecepatanDanSudutRoda(60, -270, CanIdRoda1);
//			SetKecepatanDanSudutRoda(90, 270, CanIdRoda2);
//			SetKecepatanDanSudutRoda(30, 135, CanIdRoda3);
//			HAL_Delay(1000);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, InhA_Pin|InhB_Pin|InhC_Pin|LedA_Pin
                          |LedB_Pin|LedC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SensorMagnet_Pin */
  GPIO_InitStruct.Pin = SensorMagnet_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SensorMagnet_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : InhA_Pin InhB_Pin InhC_Pin LedA_Pin
                           LedB_Pin LedC_Pin */
  GPIO_InitStruct.Pin = InhA_Pin|InhB_Pin|InhC_Pin|LedA_Pin
                          |LedB_Pin|LedC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
