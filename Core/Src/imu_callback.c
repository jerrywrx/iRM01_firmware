///*
// * imu_callback.c
// *
// *  Created on: Nov 12, 2023
// *      Author: jerrywang
// */
//
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
//	} else if (hspi->Instance == SPI3) {
//
//		if (bar.reading) {
//
//			SPL06_ReadDMA_Complete(&bar);
//
//			/* Filter barometer readings using 10-point moving average filter */
//			FIRFilter_Update(&barFilterMovingAverage, bar.pressure_Pa);
//
//		}
//
//	}
//
//}
//
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	if (GPIO_Pin == INT_ACC_Pin) {
//
//		BMI088_ReadAccelerometerDMA(&imu);
//
//	} else if (GPIO_Pin == INT_GYR_Pin) {
//
//		BMI088_ReadGyroscopeDMA(&imu);
//
//	}
//
//}
//
