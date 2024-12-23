/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define micMotor1_Pin GPIO_PIN_13
#define micMotor1_GPIO_Port GPIOC
#define micMotor1_EXTI_IRQn EXTI15_10_IRQn
#define micMotor2_Pin GPIO_PIN_0
#define micMotor2_GPIO_Port GPIOC
#define micMotor2_EXTI_IRQn EXTI0_IRQn
#define Dir_Motor2_Pin GPIO_PIN_1
#define Dir_Motor2_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOC
#define Step_Motor2_Pin GPIO_PIN_3
#define Step_Motor2_GPIO_Port GPIOC
#define En_Motor_Pin GPIO_PIN_5
#define En_Motor_GPIO_Port GPIOC
#define Step_Motor3_Pin GPIO_PIN_0
#define Step_Motor3_GPIO_Port GPIOB
#define Dir_Motor3_Pin GPIO_PIN_1
#define Dir_Motor3_GPIO_Port GPIOB
#define Analiz_Pin GPIO_PIN_2
#define Analiz_GPIO_Port GPIOB
#define laserEn_Pin GPIO_PIN_13
#define laserEn_GPIO_Port GPIOB
#define Step_Motor1_Pin GPIO_PIN_11
#define Step_Motor1_GPIO_Port GPIOA
#define Dir_Motor1_Pin GPIO_PIN_12
#define Dir_Motor1_GPIO_Port GPIOA
#define Led1_Pin GPIO_PIN_15
#define Led1_GPIO_Port GPIOA
#define Step_Motor0_Pin GPIO_PIN_10
#define Step_Motor0_GPIO_Port GPIOC
#define Dir_Motor0_Pin GPIO_PIN_11
#define Dir_Motor0_GPIO_Port GPIOC
#define micMotor0_Pin GPIO_PIN_4
#define micMotor0_GPIO_Port GPIOB
#define micMotor0_EXTI_IRQn EXTI4_IRQn
#define micMotor3_Pin GPIO_PIN_5
#define micMotor3_GPIO_Port GPIOB
#define micMotor3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
