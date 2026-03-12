// kinematics.h
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "main.h"

extern const int coefficcient_M0;
extern const int coefficcient_M1;

extern const int MAX_PWM;
extern const int MIN_PWM;
extern const int MAX_RPM;
extern const int MIN_RPM;
extern const int SELF_ROTATION_RPM;
extern const int JOYSTICK_DEADZONE_LX;
extern const int JOYSTICK_DEADZONE_LY;
extern const int TRIGGER_DEADZONE;

//extern int currentPWM_M0;
//extern int currentPWM_M1;
//extern int currentPWM_M2;
//extern int currentPWM_M3;

void InverseKinematics(int LX, int LY, int RX, int Throttle, bool isTurningLeft, bool isTurningRight);

#endif // KINEMATICS_H
