/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define AS5600_Pin GPIO_PIN_1
#define AS5600_GPIO_Port GPIOA
#define HallA_Pin GPIO_PIN_6
#define HallA_GPIO_Port GPIOA
#define HallB_Pin GPIO_PIN_7
#define HallB_GPIO_Port GPIOA
#define HallC_Pin GPIO_PIN_0
#define HallC_GPIO_Port GPIOB
#define RS485_Pin GPIO_PIN_1
#define RS485_GPIO_Port GPIOB
#define InhA_Pin GPIO_PIN_12
#define InhA_GPIO_Port GPIOB
#define InhB_Pin GPIO_PIN_14
#define InhB_GPIO_Port GPIOB
#define InhC_Pin GPIO_PIN_15
#define InhC_GPIO_Port GPIOB
#define PwmA_Pin GPIO_PIN_8
#define PwmA_GPIO_Port GPIOA
#define PwmB_Pin GPIO_PIN_9
#define PwmB_GPIO_Port GPIOA
#define PwmC_Pin GPIO_PIN_10
#define PwmC_GPIO_Port GPIOA
#define SudutMagnet_Pin GPIO_PIN_9
#define SudutMagnet_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
