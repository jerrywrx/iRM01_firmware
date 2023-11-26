#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "../BSP/Inc/bsp_print.h"
#include "../Libraries/Inc/BMI088.h"
#include "../Libraries/Inc/MahonyAHRS.h"
#include "../Libraries/Inc/controller.h"
#include "../Libraries/Inc/motor.h"
#include "../Libraries/Inc/dbus.h"

#define TARGET_SPEED 30

static BoolEdgeDetector button_init(false);

bsp::CAN* can = nullptr;
control::MotorCANBase* motor = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::Motor4310* leg_motor = nullptr;
remote::DBUS* dbus;
BMI088 imu;

static bool init = false;

/* callback triggered by an external interrupt on a GPIO pin*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == INT1_ACC_Pin) {
		BMI088_ReadAccelerometerDMA(&imu);
	} else if (GPIO_Pin == INT1_GYR_Pin) {
		BMI088_ReadGyroscopeDMA(&imu);
	}

}

/* callback function that gets called when an SPI transmit-receive operation is complete */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi->Instance == SPI1) {
		if (imu.readingAcc)
			BMI088_ReadAccelerometerDMA_Complete(&imu);	/* post-processing accelerometer data*/
		if (imu.readingGyr)
			BMI088_ReadGyroscopeDMA_Complete(&imu);	/* post-processing gyroscope data*/
	}

}


void RTOS_Init() {
	print_use_uart(&huart1);

	/* Initialise inertial measurement unit */
	BMI088_Init(&imu, &hspi1, GPIOB, SPI1_CS_ACC_Pin, GPIOA, SPI1_CS_GYRO_Pin);

	can = new bsp::CAN(&hcan, true);
	motor = new control::Motor3508(can, 0x201);
    motor2 = new control::Motor3508(can, 0x202);
    leg_motor = new control::Motor4310(can, 0x02, 0x01, control::MIT);
    dbus = new remote::DBUS(&huart2);
}


uint16_t zeroDriftTry = 1000;

void imuTask(const void* args){
	UNUSED(args);

	/* Initialise quaternion */
	imu.INS_quat[0] = 1.0f;
	imu.INS_quat[1] = 0.0f;
	imu.INS_quat[2] = 0.0f;
	imu.INS_quat[3] = 0.0f;

	/* Initialise Euler angles */
	imu.INS_euler[0] = 0.0f;
	imu.INS_euler[1] = 0.0f;
	imu.INS_euler[2] = 0.0f;

//    uint16_t count = 0;
//
//    float zeroDriftSum[3] = {0.0f, 0.0f, 0.0f};
//    float zeroDrift[3] = {0.0f, 0.0f, 0.0f};

//    while (true) {
//		init.input(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
//		if (init.negEdge()){
//			calibrated = true;
//			break;
//		}
//	}

    while (dbus->swr != remote::DOWN) {HAL_Delay(10);}
    init = true;

	while (true) {
//        if (calibrated) {
//            if (++count < zeroDriftTry) {
//                for (int i = 0; i < 3; i++) {
//                    zeroDriftSum[i] += imu.gyr_rps[i];
//                }
//                continue;
//            } else if (count == zeroDriftTry) {
//                for (int i = 0; i < 3; ++i) {
//                    zeroDrift[i] = zeroDriftSum[i] / (float)zeroDriftTry;
//                }
//                continue;
//            }
//            for (int i = 0; i < 3; ++i) {
//                imu.gyr_rps[i] -= zeroDrift[i];
//            }

            MahonyAHRSupdateIMU(imu.INS_quat, imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2], imu.acc_mps2[0],
               imu.acc_mps2[1], imu.acc_mps2[2]);

            GetEulerAngle(imu.INS_quat, &imu.INS_euler[0], &imu.INS_euler[1], &imu.INS_euler[2]);

            HAL_Delay(1);
//        }
	}
}

void chassisTask(const void* args){
	UNUSED(args);

    while (dbus->swr != remote::DOWN) {HAL_Delay(10);}
//
    control::MotorCANBase* motors[] = {motor, motor2};
    control::PIDController pid(20, 15, 30);
    control::PIDController pid2(20, 15, 30);
    control::Motor4310* leg_motors[] = {leg_motor};

    leg_motor->SetZeroPos();
    leg_motor->MotorEnable();

	while (true) {
        float vel = dbus->ch1 / 660.0 * 30.0;
        vel = clip<float>(vel, -30, 30);

        float diff = motor->GetOmegaDelta(vel);
        int16_t out = pid.ComputeConstrainedOutput(diff);
        motor->SetOutput(out);
        float diff2 = motor2->GetOmegaDelta(vel);
        int16_t out2 = pid2.ComputeConstrainedOutput(diff2);
        motor2->SetOutput(out2);
        control::MotorCANBase::TransmitOutput(motors, 2);

        leg_motor->SetOutput(0, vel, 0, 0.5, 0);
        control::Motor4310::TransmitOutput(leg_motors, 1);

        osDelay(10);
	}
}

void ledTask(const void* args){
	UNUSED(args);

	uint8_t intensity = 0;
	uint8_t add_intensity = 1;
    uint8_t pin_state = 0;

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
			add_intensity = 1;
		}

        pin_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
        if (pin_state) {
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        } else {
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        }

		HAL_Delay(50);
	}


}

void RTOS_Default_Task(const void* args) {
	UNUSED(args);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              init ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");

        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu.INS_euler[0] / PI * 180,
              imu.INS_euler[1] / PI * 180, imu.INS_euler[2] / PI * 180);

//        print("acc0: %.3f, acc1: %.3f, acc2: %.3f, gyr0: %.3f, gyr1: %.3f, gyr2: %.3f\r\n",
//            imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2]);
//
//        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
//        print("SWL: %d SWR: %d DIAL: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->wheel, dbus->timestamp);
//
//    	print("motor_start: %d\r\n", motor_start);
//        print("yaw: %.3f, pitch: %.3f, roll: %.3f\r\n", imu.INS_euler[0] / PI * 180, imu.INS_euler[1] / PI * 180, imu.INS_euler[2] / PI * 180);

        HAL_Delay(10);
    }

}
