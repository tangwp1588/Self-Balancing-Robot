#include "pid.h"
#include "motor.h"
#include "main.h"
#include "functions.h"
#include "controller_input.h"
#include "bno08x.h"

#define SAMPLE_TIME 0.01f  // 100 Hz (10ms)
#define FALLING_ANGLE 60.0f // 60 degrees
float Equilibrium_Angle = 0.0f;

BNO_Data bno_data = {0};
Desired_Data desired_data = {
	.roll = 0.0f,
	.pitch = 0.0f,
	.yaw = 0.0f,   // Orientation
	.speed = 0,
};

PidConstant pidBalancingConstant = {
	.Kp = 30.0f,
	.Ki = 0.0f,
	.Kd = 230.0f,
	.maxIntegral = 800.0f,
	.integral = 0.0f,
	.prev_error = 0.0f,
	.output = 0.0f // Common PWM
};

//Parallel PID
PidConstant pidSpeedConstant = {
	.Kp = 5.2f,
	.Ki = 1.3f,
	.Kd = 0.0f,
	.maxIntegral = 60.0f,
	.integral = 0.0f,
	.prev_error = 0.0f,
	.output = 0.0f //Desired Pitch Angle
};
// Series PID
// PidConstant pidSpeedConstant = {
// 	.Kp = 0.18f,
// 	.Ki = 0.06f,
// 	.Kd = 0.0f,
// 	.maxIntegral = 40.0f,
// 	.integral = 0.0f,
// 	.prev_error = 0.0f,
// 	.output = 0.0f //Desired Pitch Angle
// };

PidConstant pidOrientationConstant = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.maxIntegral = 180.0f,
	.integral = 0.0f,
	.prev_error = 0.0f,
	.output = 0.0f //Differential PWM
};

BNO_Data local_bno = {0};
Desired_Data local_desired_data = {0};
Motors localmotor[2] = {0};

void PID_Data_Refresh(void) {
	if(osMutexAcquire(EulerAngleMutexHandle, 1) == osOK) {
		memcpy(&local_bno, &bno_data, sizeof(BNO_Data));
		memcpy(&local_desired_data, &desired_data, sizeof(Desired_Data));
		osMutexRelease(EulerAngleMutexHandle);
	}

	if(osMutexAcquire(motorDataMutexHandle, 1) == osOK) {
		memcpy(localmotor, motor, sizeof(Motors) * 2);
		osMutexRelease(motorDataMutexHandle);
	}
}

static float filtered_gyro = 0;

void PID_Balancing(void) {
	if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {

		PidConstant* config = &pidBalancingConstant;

		if (fabs(local_bno.pitch) > FALLING_ANGLE) {
			config->integral = 0;
			config->prev_error = 0;
			config->output = 0;
			osMutexRelease(PIDValueMutexHandle);
			return;
		}

	// Round off
		// float p = roundf(local_desired_data.pitch * 10.0f) / 10.0f;
		// float eq = roundf(Equilibrium_Angle * 10.0f) / 10.0f;
		// float bno = roundf(local_bno.pitch * 10.0f) / 10.0f;
		//
		// float error = p + eq - bno;
		float error = local_desired_data.pitch + Equilibrium_Angle - local_bno.pitch;

		// Integral term
		config->integral += error * SAMPLE_TIME;

		//Anti-windup
		if (config->integral >= config->maxIntegral) config->integral = config->maxIntegral;
		if (config->integral <= -config->maxIntegral) config->integral = -config->maxIntegral;

		// Derivative term
		// float alpha = 0.2f; // Adjust between 0.1 (heavy filtering) and 0.8 (light)
		// filtered_gyro = (alpha * local_bno.gyroY) + ((1.0f - alpha) * filtered_gyro);
		// float derivative = filtered_gyro;
		float derivative = local_bno.gyroY;
		// float derivative = (error - config->prev_error) / SAMPLE_TIME;

		// PID output
		float output = (-1*config->Kp * error) +
						   (-1*config->Ki * config->integral) +
						   (-1*config->Kd * derivative);

		config->output = output;
		config->prev_error = error;
		osMutexRelease(PIDValueMutexHandle);
	}
}

float last_filtered_speed = 0.0f;

void PID_Speed(void) {
	if(osMutexAcquire(PIDValueMutexHandle, 10) != osOK) return;

	PidConstant* config = &pidSpeedConstant;

	if (fabs(local_bno.pitch) > FALLING_ANGLE) {
		config->integral = 0;
		config->prev_error = 0;
		config->output = 0;
		osMutexRelease(PIDValueMutexHandle);
		return;
	}

	float desired_speed = local_desired_data.speed;
	float alpha = 0.3f;
	float current_speed = alpha * ((localmotor[0].rpmMeasured + localmotor[1].rpmMeasured) / 2.0f) + (1 - alpha) * last_filtered_speed;
	last_filtered_speed = current_speed;

	// float current_speed = (localmotor[0].rpmMeasured + localmotor[1].rpmMeasured) / 2.0f;

	float error = desired_speed - current_speed;

	// Integral term
	config->integral += error * SAMPLE_TIME;

	//Anti-windup
	if (config->integral >= config->maxIntegral) config->integral = config->maxIntegral;
	if (config->integral <= -config->maxIntegral) config->integral = -config->maxIntegral;

	// Derivative term
	float derivative = (error - config->prev_error) / SAMPLE_TIME;

// for parallel PID (all output sum tgt for PWM)
	float output = (-1*config->Kp * error) +
					   (-1*config->Ki * config->integral) +
					   (-1*config->Kd * derivative);

	config->output = output;
	config->prev_error = error;

// for series PID (output of speed PID is fed into input of balancing PID)
	// float output = (config->Kp * error) +
	// 					   (config->Ki * config->integral) +
	// 					   (config->Kd * derivative);
	//
	// config->output = output;
	// config->prev_error = error;
	// if(osMutexAcquire(EulerAngleMutexHandle, 2) == osOK) {
	// 	desired_data.pitch = output;
	// 	osMutexRelease(EulerAngleMutexHandle);
	// }

	// don't comment out this accidentally
	osMutexRelease(PIDValueMutexHandle);
}

void PID_Orientation(void) {
	if(osMutexAcquire(PIDValueMutexHandle, 10) != osOK) return;

	PidConstant* config = &pidOrientationConstant;

	if (fabs(local_bno.pitch) > FALLING_ANGLE) {
		config->integral = 0;
		config->prev_error = 0;
		config->output = 0;
		osMutexRelease(PIDValueMutexHandle);
		return;
	}

	float error = local_desired_data.yaw - local_bno.yaw;

	// Handle angle wrap-around from -180 to +180 degrees
	if (error > 180.0f) {
		error -= 360.0f;
	} else if (error < -180.0f) {
		error += 360.0f;
	}

	// Integral term
	config->integral += error * SAMPLE_TIME;

	//Anti-windup
	if (config->integral >= config->maxIntegral) config->integral = config->maxIntegral;
	if (config->integral <= -config->maxIntegral) config->integral = -config->maxIntegral;

	// Derivative term
	float derivative = local_bno.gyroZ;
	// PID output
	float output = (config->Kp * error) +
					   (config->Ki * config->integral) +
					   (config->Kd * derivative);

	config->output = output;
	// config->prev_error = error;

	osMutexRelease(PIDValueMutexHandle);
}
