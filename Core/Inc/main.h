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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088.h"
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
void RTOS_Init(void);
void RTOS_Default_Task(const void *argument);
void imuTask(const void *argument);
void ledTask(const void *argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_ACC_Pin GPIO_PIN_15
#define SPI1_CS_ACC_GPIO_Port GPIOB
#define SPI1_CS_GYRO_Pin GPIO_PIN_8
#define SPI1_CS_GYRO_GPIO_Port GPIOA
#define INT1_GYR_Pin GPIO_PIN_15
#define INT1_GYR_GPIO_Port GPIOA
#define INT1_GYR_EXTI_IRQn EXTI15_10_IRQn
#define INT1_ACC_Pin GPIO_PIN_5
#define INT1_ACC_GPIO_Port GPIOB
#define INT1_ACC_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
