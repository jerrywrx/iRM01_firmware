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
#include "balance.h"

static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* left_wheel_motor = nullptr;
static control::MotorCANBase* right_wheel_motor = nullptr;
static control::Motor4310* left_front_leg_motor = nullptr;
static control::Motor4310* left_back_leg_motor = nullptr;
static control::Motor4310* right_front_leg_motor = nullptr;
static control::Motor4310* right_back_leg_motor = nullptr;
static remote::DBUS* dbus = nullptr;
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
	/* Initialise inertial measurement unit */
	BMI088_Init(&imu, &hspi1, GPIOB, SPI1_CS_ACC_Pin, GPIOA, SPI1_CS_GYRO_Pin);

    //dbus and print initialization
    print_use_uart(&huart1);
    dbus = new remote::DBUS(&huart2);

    // can initialization
    can1 = new bsp::CAN(&hcan, true);

    // wheel motor initialization
    left_wheel_motor = new control::Motor3508(can1, 0x201);
    right_wheel_motor = new control::Motor3508(can1, 0x202);

    // leg motor initialization
    left_front_leg_motor = new control::Motor4310(can1, 0x02, 0x01, control::MIT);
    left_back_leg_motor = new control::Motor4310(can1, 0x04, 0x03, control::MIT);
    right_front_leg_motor = new control::Motor4310(can1, 0x06, 0x05, control::MIT);
    right_back_leg_motor = new control::Motor4310(can1, 0x08, 0x07, control::MIT);
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

control::ConstrainedPID* balance_controller = new control::ConstrainedPID(500.0, 0.0, 100.0, 14000.0, 14000.0);
control::ConstrainedPID* velocity_controller = new control::ConstrainedPID(10000.0, 0.0, 2000.0, 14000.0, 14000.0);
control::ConstrainedPID* rotation_controller = new control::ConstrainedPID(3000.0, 0.0, 500.0, 14000.0, 14000.0);
control::ConstrainedPID* left_wheel_vel_pid = new control::ConstrainedPID(2000, 10, 500, 14000.0, 14000.0);
control::ConstrainedPID* right_wheel_vel_pid = new control::ConstrainedPID(2000, 10, 500, 14000.0, 14000.0);
float balance_output = 0.0;
float velocity_output = 0.0;
float rotation_output = 0.0;

void chassisTask(const void* args){
    UNUSED(args);

    // start switch
    while(true){
        if (dbus->swr == remote::DOWN) {
            break;
        }
        osDelay(100);
    }

    control::MotorCANBase* wheel_motors[] = {left_wheel_motor, right_wheel_motor};
//    control::Motor4310* leg_motor1[] = {left_front_leg_motor, left_back_leg_motor, right_front_leg_motor, right_back_leg_motor};
    control::Motor4310* leg_motor1[] = {left_front_leg_motor};
    control::Motor4310* leg_motor2[] = {left_back_leg_motor};
    control::Motor4310* leg_motor3[] = {right_front_leg_motor};
    control::Motor4310* leg_motor4[] = {right_back_leg_motor};

    osDelay(10);
    print("Calibration done\r\n");
    osDelay(1000);
    // leg motor activation
    //  left_front_leg_motor->SetZeroPos();
    left_front_leg_motor->MotorEnable();
    osDelay(100);
    //  left_back_leg_motor->SetZeroPos();
    left_back_leg_motor->MotorEnable();
    osDelay(100);
    //  right_front_leg_motor->SetZeroPos();
    right_front_leg_motor->MotorEnable();
    osDelay(100);
    //  right_back_leg_motor->SetZeroPos();
    right_back_leg_motor->MotorEnable();

    while (dbus->swl != remote::DOWN) {osDelay(100);}

    left_front_leg_motor->SetRelativeTarget(0);
    left_back_leg_motor->SetRelativeTarget(0);
    right_front_leg_motor->SetRelativeTarget(0);
    right_back_leg_motor->SetRelativeTarget(0);

    for (int j = 0; j < 300; j++){
        left_front_leg_motor->SetRelativeTarget(left_front_leg_motor->GetRelativeTarget() + demo_height / 300);  // increase position gradually
        left_front_leg_motor->SetOutput(left_front_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
        left_back_leg_motor->SetRelativeTarget(left_back_leg_motor->GetRelativeTarget() + demo_height / 300);
        left_back_leg_motor->SetOutput(-left_back_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
        right_front_leg_motor->SetRelativeTarget(right_front_leg_motor->GetRelativeTarget() + demo_height / 300);
        right_front_leg_motor->SetOutput(-right_front_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
        right_back_leg_motor->SetRelativeTarget(right_back_leg_motor->GetRelativeTarget() + demo_height / 300);
        right_back_leg_motor->SetOutput(right_back_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);

        control::Motor4310::TransmitOutput(leg_motor1, 1);
        osDelay(1);
        control::Motor4310::TransmitOutput(leg_motor2, 1);
        osDelay(1);
        control::Motor4310::TransmitOutput(leg_motor3, 1);
        osDelay(1);
        control::Motor4310::TransmitOutput(leg_motor4, 1);
        osDelay(1);
    }
    float pos = demo_height;
    float min_pos = -PI/4;
    float max_pos = PI/4;

    float balance_angle = 0; // measured by IMU
    float left_output = 0.0;
    float right_output = 0.0;
    float balance_difference = 0.0;
    float velocity_difference = 0.0;
    float rotation_difference = 0.0;
    current_raw_yaw = imu.INS_euler[0];
    last_raw_yaw = imu.INS_euler[0];
    rotation_angle = current_raw_yaw;

    while (true) {
        // position control
        balance_difference =  -(balance_angle - imu.INS_euler[1] / PI * 180);
        // velocity control
        velocity_difference = (dbus->ch1/660.0)
                              - (-left_wheel_motor->GetOmega() + right_wheel_motor->GetOmega()) * rpm_rads * wheel_radius * 0.5;
        // rotation control
        rotation_angle += -(dbus->ch2/660.0) * 0.3;
        last_raw_yaw = current_raw_yaw;
        current_raw_yaw = imu.INS_euler[0];
        current_yaw = turn_count * 2 * PI + current_raw_yaw;
        if (fabs(current_raw_yaw - last_raw_yaw) > 1.9f * PI)
        {
            if ((current_raw_yaw - last_raw_yaw) < 0)
                turn_count++;
            else
                turn_count--;
        }

        rotation_difference = rotation_angle - current_yaw;

        balance_output = -balance_controller->ComputeOutput(balance_difference);
        velocity_output = velocity_controller->ComputeOutput(velocity_difference);
        rotation_output = rotation_controller->ComputeOutput(rotation_difference);
        if (dbus->swr == remote::DOWN || imu.INS_euler[1] / PI * 180 < balance_angle - 75 || imu.INS_euler[1] / PI * 180 > balance_angle + 75
            || left_wheel_motor->GetCurr() > 16384 || right_wheel_motor->GetCurr() > 16384) {
            left_output = 0;
            right_output = 0;
        } else {
            left_output = balance_output + velocity_output + rotation_output;
            right_output = balance_output + velocity_output - rotation_output;

            left_output = left_wheel_vel_pid->ComputeOutput(left_output * magic_rpm - left_wheel_motor->GetOmega());
            right_output = right_wheel_vel_pid->ComputeOutput(right_output * magic_rpm + right_wheel_motor->GetOmega());

            left_output = clip<float>(left_output, -14000, 14000);
            right_output = clip<float>(right_output, -14000, 14000);
        }
        left_wheel_motor->SetOutput((int16_t)left_output);
        right_wheel_motor->SetOutput(-(int16_t)right_output);

        if (dbus->swl == remote::MID) {
            float vel;
            vel = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15);
            pos += vel / 200;
            pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

            left_front_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);
            left_back_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
            right_front_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
            right_back_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);
        } else if (dbus->swl == remote::UP && jump_flag == true) {
            jump_flag = false;
            left_front_leg_motor->SetOutput(jump_height, 15, 30, 3, 0);
            left_back_leg_motor->SetOutput(-jump_height, -15, 30, 3, 0);
            right_front_leg_motor->SetOutput(-jump_height, -15, 30, 3, 0);
            right_back_leg_motor->SetOutput(jump_height, 15, 30, 3, 0);
//            control::Motor4310::TransmitOutput(leg_motors, 4);
            control::Motor4310::TransmitOutput(leg_motor1, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor2, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor3, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor4, 1);
            osDelay(1);
            osDelay(180);
            left_front_leg_motor->SetOutput(pos, 15, 30, 3, 0);
            left_back_leg_motor->SetOutput(-pos, -15, 30, 3, 0);
            right_front_leg_motor->SetOutput(-pos, -15, 30, 3, 0);
            right_back_leg_motor->SetOutput(pos, 15, 30, 3, 0);
//            control::Motor4310::TransmitOutput(leg_motors, 4);
            control::Motor4310::TransmitOutput(leg_motor1, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor2, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor3, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor4, 1);
            osDelay(1);
            osDelay(5);
        }

        if (dbus->swr == remote::UP) {
            left_wheel_motor->SetOutput(0);
            right_wheel_motor->SetOutput(0);
            left_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
            left_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
            right_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
            right_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
        }

        control::MotorCANBase::TransmitOutput(wheel_motors, 2);
        if (dbus->swl != remote::UP) {
            jump_flag = true;
//            control::Motor4310::TransmitOutput(leg_motors, 4);
            control::Motor4310::TransmitOutput(leg_motor1, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor2, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor3, 1);
            osDelay(1);
            control::Motor4310::TransmitOutput(leg_motor4, 1);
            osDelay(1);
        }
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
