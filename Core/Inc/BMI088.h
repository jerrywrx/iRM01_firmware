#ifndef BMI088_IMU_H
#define BMI088_IMU_H

#include "stm32f1xx_hal.h"
#include <math.h>

/* Register defines */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_DATA 			0x12
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csAccPinBank;
	GPIO_TypeDef 	  *csGyrPinBank;
	uint16_t 		   csAccPin;
	uint16_t 		   csGyrPin;

	/* DMA */
	uint8_t readingAcc;
	uint8_t readingGyr;
	uint8_t accTxBuf[8];
	uint8_t gyrTxBuf[7];
	volatile uint8_t accRxBuf[8];
	volatile uint8_t gyrRxBuf[7];

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float accConversion;
	float gyrConversion;

	/* x-y-z measurements */
	float acc_mps2[3];
	float gyr_rps[3];

	float INS_quat[4];
	float INS_euler[3];

} BMI088;

/****** INITIALISATION ******/
uint8_t BMI088_Init(BMI088 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				 GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin);

/****** LOW-LEVEL REGISTER FUNCTIONS ******/
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

/****** POLLING ******/
uint8_t BMI088_ReadAccelerometer(BMI088 *imu);
uint8_t BMI088_ReadGyroscope(BMI088 *imu);

/****** DMA ******/

/* Start accelerometer DMA transaction */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu);
/* Data processing after DMA transaction is complete*/
void 	BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu);

/* Start gyroscope DMA transaction */
uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu);
/* Data processing after DMA transaction is complete*/
void 	BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu);

/****** HELPER FUNCTIONS ******/
void GetEulerAngle(float* q, float* yaw, float* pitch, float* roll);

#endif
