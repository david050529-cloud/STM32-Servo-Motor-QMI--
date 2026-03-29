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
#define Coder1_CH1_Pin GPIO_PIN_0
#define Coder1_CH1_GPIO_Port GPIOA
#define Coder1_CH2_Pin GPIO_PIN_1
#define Coder1_CH2_GPIO_Port GPIOA
#define Motor2_CH2_Pin GPIO_PIN_9
#define Motor2_CH2_GPIO_Port GPIOE
#define Motor2_CH1_Pin GPIO_PIN_11
#define Motor2_CH1_GPIO_Port GPIOE
#define Motor1_CH2_Pin GPIO_PIN_13
#define Motor1_CH2_GPIO_Port GPIOE
#define Motor1_CH1_Pin GPIO_PIN_14
#define Motor1_CH1_GPIO_Port GPIOE
#define IMU_ITR_Pin GPIO_PIN_12
#define IMU_ITR_GPIO_Port GPIOB
#define IMU_ITR_EXTI_IRQn EXTI15_10_IRQn
#define MASTER_TX_Pin GPIO_PIN_8
#define MASTER_TX_GPIO_Port GPIOD
#define MASTER_RX_Pin GPIO_PIN_9
#define MASTER_RX_GPIO_Port GPIOD
#define PWM_SERVO_TIM_Pin GPIO_PIN_8
#define PWM_SERVO_TIM_GPIO_Port GPIOC
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOA
#define PWM_SERVO_2_Pin GPIO_PIN_12
#define PWM_SERVO_2_GPIO_Port GPIOA
#define Coder2_CH1_Pin GPIO_PIN_15
#define Coder2_CH1_GPIO_Port GPIOA
#define Coder2_CH2_Pin GPIO_PIN_3
#define Coder2_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
