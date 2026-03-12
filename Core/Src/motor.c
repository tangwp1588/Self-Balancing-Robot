#include "motor.h"
#include "functions.h"
#include "kinematics.h"
#include "main.h"       // For HAL/GPIO functions
#include "tim.h"

//  when pid is activated, the pins are correct but the motor runs forever once start, exchange the two ENCODER pins to fix it!
//  This is because motor runs +ve rpm but encoder reads -ve rpm, and this cause the pid to windup.
// sometimes you also need to exchange the encoder pins after you modified motor Coefficient

void motor_INIT (void) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // M0(Left)
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // M1 (Actually is M3 from the mechanum wheel code)
}

void controlMotor(Motors *motors, int count) {
	if(osMutexAcquire(motorDataMutexHandle, 2) != osOK) return;

	// Copy motor data
	Motors local_motors[count];
	memcpy(local_motors, motors, sizeof(Motors) * count);

	osMutexRelease(motorDataMutexHandle);

	int id, speed;
	for (id = 0; id < count; id++) {
		switch(id) {
			case 0:
				speed = coefficcient_M0 * local_motors[0].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, constrain(abs(speed) + MIN_PWM, 0, MAX_PWM));
				break;

			case 1:
				speed = coefficcient_M1 * local_motors[1].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, constrain(abs(speed) + MIN_PWM, 0, MAX_PWM));
				break;
			default:
				speed = 0;
				HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, RESET);
				HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, speed);
				break;
		}
	}
}


//void stopAllMotors(void) {
//	controlMotor(0, 0);
//	controlMotor(1, 0);
//	controlMotor(2, 0);
//	controlMotor(3, 0);
//}

