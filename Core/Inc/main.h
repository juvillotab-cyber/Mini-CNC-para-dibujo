/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B2_Z_Pin GPIO_PIN_0
#define B2_Z_GPIO_Port GPIOA
#define B2_Z_EXTI_IRQn EXTI0_1_IRQn
#define B1_Z_Pin GPIO_PIN_1
#define B1_Z_GPIO_Port GPIOA
#define B1_Z_EXTI_IRQn EXTI0_1_IRQn
#define M_Z1_Pin GPIO_PIN_6
#define M_Z1_GPIO_Port GPIOA
#define M_Z2_Pin GPIO_PIN_7
#define M_Z2_GPIO_Port GPIOA
#define M_Z3_Pin GPIO_PIN_0
#define M_Z3_GPIO_Port GPIOB
#define MZ_4_Pin GPIO_PIN_1
#define MZ_4_GPIO_Port GPIOB
#define M_Y4_Pin GPIO_PIN_12
#define M_Y4_GPIO_Port GPIOB
#define M_Y3_Pin GPIO_PIN_13
#define M_Y3_GPIO_Port GPIOB
#define M_Y2_Pin GPIO_PIN_14
#define M_Y2_GPIO_Port GPIOB
#define M_Y1_Pin GPIO_PIN_15
#define M_Y1_GPIO_Port GPIOB
#define M_X4_Pin GPIO_PIN_8
#define M_X4_GPIO_Port GPIOA
#define M_X3_Pin GPIO_PIN_9
#define M_X3_GPIO_Port GPIOA
#define M_X2_Pin GPIO_PIN_10
#define M_X2_GPIO_Port GPIOA
#define M_X1_Pin GPIO_PIN_11
#define M_X1_GPIO_Port GPIOA
#define B_XP_Pin GPIO_PIN_15
#define B_XP_GPIO_Port GPIOA
#define B_XP_EXTI_IRQn EXTI4_15_IRQn
#define B_XN_Pin GPIO_PIN_3
#define B_XN_GPIO_Port GPIOB
#define B_XN_EXTI_IRQn EXTI2_3_IRQn
#define B_YP_Pin GPIO_PIN_4
#define B_YP_GPIO_Port GPIOB
#define B_YP_EXTI_IRQn EXTI4_15_IRQn
#define B_YN_Pin GPIO_PIN_5
#define B_YN_GPIO_Port GPIOB
#define B_YN_EXTI_IRQn EXTI4_15_IRQn
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
