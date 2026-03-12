#include "controller_input.h"
#include "motor.h"
#include "kinematics.h"
#include "uart_handler.h"
#include "main.h"
#include "pid.h"
#include "bno08x.h"

bool enableBrake = false;
bool isTurningLeft = false;
bool isTurningRight = false;
bool isSelfRotate = false;
bool enableOrientationPID = false;

void processGamepad(void) {
	ControllerData* data = UART_GetControllerData();

	enableBrake = data->brake > 500;
	if (data->brake > 500) {
		motor[0].pwm = 0;
		motor[1].pwm = 0;
		return;
	}

    if (data->buttons & BUTTON_X) {
    	// X = FL
    	motor[0].pwm = 499;
        return;
	}

	if (data->buttons & BUTTON_Y) {
		// Y = FR
		motor[1].pwm = 499;
		return;
	}

	// if (data->buttons & BUTTON_A) {
	// 	// A = BL
	// 	motor[3].rpmDesired = 120;
	// 	return;
	// }
	//
	// if (data->buttons & BUTTON_B) {
	// 	// B = BR
	// 	motor[2].rpmDesired = 120;
	// 	return;
	// }

	static uint8_t lastButtonAState = 0;
	uint8_t currentButtonAState = data->buttons & BUTTON_A;

	if (currentButtonAState && !lastButtonAState) { // On button A press (rising edge)
		osDelay(20); // Debounce
		if (data->buttons & BUTTON_A) {
			enableOrientationPID = !enableOrientationPID;

			if (enableOrientationPID) {
				// Orientation PID was just enabled, record the current yaw as the setpoint.
				if(osMutexAcquire(EulerAngleMutexHandle, 2) == osOK) {
					desired_data.yaw = bno_data.yaw; // Set the target yaw in the PID controller
					osMutexRelease(EulerAngleMutexHandle);
				}
			}

			// Wait for button release to prevent multiple toggles from a single press
			while (data->buttons & BUTTON_A) {
				osDelay(10);
			}
		}
	}
	lastButtonAState = currentButtonAState;

	isTurningLeft = data->buttons & BUTTON_LB;
	isTurningRight = data->buttons & BUTTON_RB;
	isSelfRotate = abs(data->axisRX) > 50;

	if (data->dpad & DPAD_UP) {
		InverseKinematics(0, -508, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
	} else if (data->dpad & DPAD_DOWN) {
		InverseKinematics(0, 508, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
	} else if (data->dpad & DPAD_LEFT) {
		InverseKinematics(-508, 0, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
	} else if (data->dpad & DPAD_RIGHT) {
		InverseKinematics(508, 0, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
	} else {
		InverseKinematics(data->axisX, data->axisY, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
	}
	// InverseKinematics(data->axisX, data->axisY, data->axisRX, data->throttle, isTurningLeft, isTurningRight);
}
