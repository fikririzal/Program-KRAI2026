/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HallA_Pin GPIO_PIN_6
#define HallA_GPIO_Port GPIOA
#define HallB_Pin GPIO_PIN_7
#define HallB_GPIO_Port GPIOA
#define HallC_Pin GPIO_PIN_0
#define HallC_GPIO_Port GPIOB
#define INA_Pin GPIO_PIN_12
#define INA_GPIO_Port GPIOB
#define INB_Pin GPIO_PIN_14
#define INB_GPIO_Port GPIOB
#define INC_Pin GPIO_PIN_15
#define INC_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define PWMC_Pin GPIO_PIN_10
#define PWMC_GPIO_Port GPIOA
#define LedHallA_Pin GPIO_PIN_5
#define LedHallA_GPIO_Port GPIOB
#define LedHallB_Pin GPIO_PIN_6
#define LedHallB_GPIO_Port GPIOB
#define LedHallC_Pin GPIO_PIN_7
#define LedHallC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
