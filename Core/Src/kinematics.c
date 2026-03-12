#include "kinematics.h"

#include "controller_input.h"
#include "functions.h"
#include "motor.h"
#include "main.h"
#include "pid.h"

const int coefficcient_M0 = 1;
const int coefficcient_M1 = -1;

const int MAX_PWM = 999;
const int MIN_PWM = 0;

const int MIN_RPM = 0;
const int MAX_RPM = 125;
const int TURNING_RPM = 100;

const int SELF_ROTATION_RPM = 120;
const int JOYSTICK_DEADZONE_LX = 50;
const int JOYSTICK_DEADZONE_LY = 25;
const int TRIGGER_DEADZONE = 0;

Motors motor[2] = {0};// the actual location of this array

int speed = 0;
void InverseKinematics(int LX, int LY, int RX, int Throttle, bool isTurningLeft, bool isTurningRight) {
	int x = (abs(LX) < JOYSTICK_DEADZONE_LX) ? 0 : LX;
	int y = (abs(LY) < JOYSTICK_DEADZONE_LY) ? 0 : -LY;
	// int rx = (abs(RX) < JOYSTICK_DEADZONE_LX) ? 0 : map(RX, -512, 508, -SELF_ROTATION_RPM, SELF_ROTATION_RPM);
	// int rx = (abs(RX) < JOYSTICK_DEADZONE_LX) ? 0 : map(RX, -512, 508, -6000, 6000); // for direct-pwm

	// Calculate angle and power from controller inputs
	// double theta;
	// if (x == 0 && y == 0) theta = 0.5 * M_PI;
	// else theta = atan2(y, x);

	int power = (Throttle < TRIGGER_DEADZONE) ? 0 : map(Throttle, TRIGGER_DEADZONE, 1020,MIN_RPM, MAX_RPM);
	// int power = (Throttle < TRIGGER_DEADZONE) ? 0 : map(Throttle, TRIGGER_DEADZONE, 1020, 0, MAX_PWM-2000); // for direct-pwm

	// Convert from polar to mechanum drive equations
	// double Sin = sin(theta - M_PI / 4);
	// double Cos = cos(theta - M_PI / 4);
	// double Max = (fabs(Sin) > fabs(Cos)) ? fabs(Sin) : fabs(Cos);
	//
	// int speed_M0 = (isTurningLeft)  ? 0 : round(power * Cos / Max) + rx; // FRONT_LEFT
	// int speed_M1 = (isTurningRight) ? 0 : round(power * Sin / Max) - rx; // FRONT_RIGHT

	if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
	// Normal Speed Control
		// if (y < 0) {
		// 	desired_data.speed = -power;
		// } else {
		// 	desired_data.speed = power;
		// }

	// Speed Control with Ramp
		if (y < 0) {
			if (abs(x) > 125 && abs(power) > TURNING_RPM) {
				speed = -TURNING_RPM;
			} else speed = -power;
		} else {
			if (abs(x) > 125 && abs(power) > TURNING_RPM) {
				speed = TURNING_RPM;
			} else speed = power;
		}

		if (osMutexAcquire(EulerAngleMutexHandle, 2) == osOK) {
			int16_t error = speed - desired_data.speed; // Distance to target
			int16_t step = 2;

			if (abs(error) <= step) {
				desired_data.speed = speed; // Close enough, just snap to it
			} else {
				// Move by 2 in the correct direction
				desired_data.speed += (error > 0) ? step : -step;
			}
			osMutexRelease(EulerAngleMutexHandle);
		}

	// for parallel PID (all output sum tgt)
		if (osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
			if (x > 125) {
				motor[0].pwm = (int16_t)roundf(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output);
				motor[1].pwm = (int16_t)roundf(0.6f*(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output));
			} else if (x < -125) {
				motor[0].pwm = (int16_t)roundf(0.6f*(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output));
				motor[1].pwm = (int16_t)roundf(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output);
			} else {
				motor[0].pwm = (int16_t)roundf(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output);
				motor[1].pwm = (int16_t)roundf(pidBalancingConstant.output + pidSpeedConstant.output + pidOrientationConstant.output);
			}
			osMutexRelease(PIDValueMutexHandle);
		}

	// for series PID (output of speed PID is feeded into input of balancing PID)
		// if (osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
		// 	if (x > 125) {
		// 		motor[0].pwm = (int16_t)roundf(pidBalancingConstant.output + pidOrientationConstant.output);
		// 		motor[1].pwm = (int16_t)roundf(0.5f*(pidBalancingConstant.output + pidOrientationConstant.output));
		// 	} else if (x < -125) {
		// 		motor[0].pwm = (int16_t)roundf(0.5f*(pidBalancingConstant.output + pidOrientationConstant.output));
		// 		motor[1].pwm = (int16_t)roundf(pidBalancingConstant.output + pidOrientationConstant.output);
		// 	} else {
		// 		motor[0].pwm = (int16_t)roundf(pidBalancingConstant.output + pidOrientationConstant.output);
		// 		motor[1].pwm = (int16_t)roundf(pidBalancingConstant.output + pidOrientationConstant.output);
		// 	}
		// 	osMutexRelease(PIDValueMutexHandle);
		// }

		// don't comment out this accidentally
		osMutexRelease(motorDataMutexHandle);
	}
}
