/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

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
#define M1_CHA_Pin GPIO_PIN_0
#define M1_CHA_GPIO_Port GPIOA
#define M1_CHB_Pin GPIO_PIN_1
#define M1_CHB_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define M1_PWM_Pin GPIO_PIN_3
#define M1_PWM_GPIO_Port GPIOA
#define M1_INA_Pin GPIO_PIN_4
#define M1_INA_GPIO_Port GPIOA
#define M1_INB_Pin GPIO_PIN_5
#define M1_INB_GPIO_Port GPIOA
#define M2_CHA_Pin GPIO_PIN_8
#define M2_CHA_GPIO_Port GPIOA
#define M2_CHB_Pin GPIO_PIN_9
#define M2_CHB_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_11
#define M2_PWM_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_3
#define USER_LED_GPIO_Port GPIOB
#define M2_INB_Pin GPIO_PIN_4
#define M2_INB_GPIO_Port GPIOB
#define M2_INA_Pin GPIO_PIN_5
#define M2_INA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */