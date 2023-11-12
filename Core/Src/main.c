/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088.h"
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

/* USER CODE BEGIN PV */
//BMI088 imu;
//SPI_HandleTypeDef hspi1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//
//	if (hspi->Instance == SPI1) {
//
//		if (imu.readingAcc) {
//
//			BMI088_ReadAccelerometerDMA_Complete(&imu);
//
//			/* Filter accelerometer data */
//		//	RCFilter_Update(&lpfAcc[0], imu.acc_mps2[0]);
//		//	RCFilter_Update(&lpfAcc[1], imu.acc_mps2[1]);
//		//	RCFilter_Update(&lpfAcc[2], imu.acc_mps2[2]);
//
//		}
//
//		if (imu.readingGyr) {
//
//			BMI088_ReadGyroscopeDMA_Complete(&imu);
//
//			/* Filter gyroscope data */
//		//	RCFilter_Update(&lpfGyr[0], imu.gyr_rps[0]);
//		//	RCFilter_Update(&lpfGyr[1], imu.gyr_rps[1]);
//		//	RCFilter_Update(&lpfGyr[2], imu.gyr_rps[2]);
//
//		}
//
//	}
//
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	if (GPIO_Pin == INT1_ACC_Pin) {
//
//		BMI088_ReadAccelerometerDMA(&imu);
//
//	} else if (GPIO_Pin == INT1_GYR_Pin) {
//
//		BMI088_ReadGyroscopeDMA(&imu);
//
//	}
//
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  print_use_uart(&huart1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//  uint8_t intensity = 0;
//  uint8_t add_intensity = 1;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (add_intensity) {
//		  intensity += 5;
//	  } else {
//		  intensity -= 5;
//	  }
//
//	  htim3.Instance->CCR4 = 100 - intensity;
//
//	  if (intensity >= 100) {
//		  add_intensity = 0;
//	  } else if (intensity <= 0) {
//		  HAL_Delay(2000);
//		  add_intensity = 1;
//	  }
//
////	  print("Intensity: %d\r\n", intensity);
//
//	  HAL_Delay(50);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//uint8_t BMI088_GyroRead(uint_8_t addr, uint8_t* data) {
//  uint8_t txbuf[2] = {(addr | 0x80), 0};
//  uint8_t rxbuf[2];
//
//  HAL_GPIO_WritePin(SPI1_CS_GYRO_GPIO_Port, SPI1_CS_GYRO_Pin, GPIO_PIN_RESET);
//  uint8_t status = (HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, 2, HAL_MAX_DELAY) == HAL_OK);
//  HAL_GPIO_WritePin(SPI1_CS_GYRO_GPIO_Port, SPI1_CS_GYRO_Pin, GPIO_PIN_SET);
//
//  *data = rxbuf[1];
//
//  return status;
//}
//
//uint8_t BMI088_GyroWrite(uint_8_t addr, uint8_t data) {
//  uint8_t txbuf[2] = {addr, data};
//
//  HAL_GPIO_WritePin(SPI1_CS_GYRO_GPIO_Port, SPI1_CS_GYRO_Pin, GPIO_PIN_RESET);
//  uint8_t status = (HAL_SPI_Transmit(&hspi1, txbuf, rxbuf, 2, HAL_MAX_DELAY) == HAL_OK);
//  HAL_GPIO_WritePin(SPI1_CS_GYRO_GPIO_Port, SPI1_CS_GYRO_Pin, GPIO_PIN_SET);
//
//  return status;
//}

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
