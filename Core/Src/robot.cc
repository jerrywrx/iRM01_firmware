#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "bsp_print.h"
#include "BMI088.h"



void RTOS_Init() {
	print_use_uart(&huart1);

//	BMI088_Init(&imu, &hspi1, GPIOA, SPI1_NCS_ACC_Pin, GPIOC, SPI1_NCS_GYR_Pin);
}

void RTOS_Default_Task(const void* args) {
	UNUSED(args);

	uint8_t intensity = 0;
	uint8_t add_intensity = 1;

	while (true) {
		if (add_intensity) {
			intensity += 5;
		} else {
			intensity -= 5;
		}

		htim3.Instance->CCR4 = 100 - intensity;

		if (intensity >= 100) {
			add_intensity = 0;
		} else if (intensity <= 0) {
			HAL_Delay(500);
			add_intensity = 1;
		}

		print("Intensity: %d\r\n", intensity);

		HAL_Delay(50);
	}

//	while (true) {
//		print("test\r\n");
//		print("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", imu->acc_mps2[0], imu->acc_mps2[1], imu->acc_mps2[2],
//							  	  	  	  	  	  	  	  	  	  	  	   imu->gyr_rps[0], imu->gyr_rps[1], imu->gyr_rps[2]);
//	}

}
