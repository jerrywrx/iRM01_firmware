#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "bsp_print.h"
#include "BMI088.h"
#include "spi.h"

BMI088 imu;
//
///* callback triggered by an external interrupt on a GPIO pin*/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	if (GPIO_Pin == INT1_ACC_Pin) {
//		BMI088_ReadAccelerometerDMA(&imu);
//	} else if (GPIO_Pin == INT1_GYR_Pin) {
//		BMI088_ReadGyroscopeDMA(&imu);
//	}
//
//}
//
///* callback function that gets called when an SPI transmit-receive operation is complete */
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//
//	if (hspi->Instance == SPI1) {
//		if (imu.readingAcc) {
//			BMI088_ReadAccelerometerDMA_Complete(&imu);	/* post-processing accelerometer data*/
//
//			/* Filter accelerometer data */
//		//	RCFilter_Update(&lpfAcc[0], imu.acc_mps2[0]);
//		//	RCFilter_Update(&lpfAcc[1], imu.acc_mps2[1]);
//		//	RCFilter_Update(&lpfAcc[2], imu.acc_mps2[2]);
//		}
//		if (imu.readingGyr) {
//			BMI088_ReadGyroscopeDMA_Complete(&imu);	/* post-processing gyroscope data*/
//
//			/* Filter gyroscope data */
//		//	RCFilter_Update(&lpfGyr[0], imu.gyr_rps[0]);
//		//	RCFilter_Update(&lpfGyr[1], imu.gyr_rps[1]);
//		//	RCFilter_Update(&lpfGyr[2], imu.gyr_rps[2]);
//		}
//	}
//}


void RTOS_Init() {
	print_use_uart(&huart1);
	BMI088_Init(&imu, &hspi1, GPIOB, SPI1_CS_ACC_Pin, GPIOA, SPI1_CS_GYRO_Pin);
}

void RTOS_Default_Task(const void* args) {
	UNUSED(args);

//	uint8_t intensity = 0;
//	uint8_t add_intensity = 1;
//
//	while (true) {
//		if (add_intensity) {
//			intensity += 5;
//		} else {
//			intensity -= 5;
//		}
//
//		htim3.Instance->CCR4 = 100 - intensity;
//
//		if (intensity >= 100) {
//			add_intensity = 0;
//		} else if (intensity <= 0) {
//			HAL_Delay(500);
//			add_intensity = 1;
//		}
//
//		print("Intensity: %d\r\n", intensity);
//
//		HAL_Delay(50);
//	}

	while (true) {
//		print("test\r\n");
//		print("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2]);

//		uint8_t acc_chip_id;
//		BMI088_ReadAccRegister(&imu, 0x00, &acc_chip_id);
//		print("%d\r\n", acc_chip_id);
//
//		uint8_t gyr_chip_id;
//		BMI088_ReadGyrRegister(&imu, 0x00, &gyr_chip_id);
//		print("%d\r\n", gyr_chip_id);
//
//		uint8_t acc_z_msb;
//		BMI088_ReadAccRegister(&imu, 0x17, &acc_z_msb);
//		print("%d\r\n", acc_z_msb);
//
//		uint8_t gyr_z_msb;
//		BMI088_ReadGyrRegister(&imu, 0x07, &gyr_z_msb);
//		print("%d\r\n", gyr_z_msb);

		BMI088_ReadAccelerometer(&imu);
		BMI088_ReadGyroscope(&imu);

//		clear_screen();
//		set_cursor(0, 0);
		print("acc0: %.3f, acc1: %.3f, acc2: %.3f, gyr0: %.3f, gyr1: %.3f, gyr2: %.3f\r\n", imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2]);

//		print("%.3f\r\n", imu.acc_mps2[0]);
		HAL_Delay(10);
	}

}
