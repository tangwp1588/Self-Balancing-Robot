// controller_input.h
#pragma once
#include "main.h"       // For HAL/GPIO functions
#include "cmsis_os2.h"

#pragma pack(push, 1)
typedef struct {
    int16_t axisX, axisY, axisRX, axisRY;
    uint16_t dpad;
    uint16_t buttons;
    uint16_t brake, throttle;
} ControllerData;
#pragma pack(pop)

// D-pad bitmasks
#define DPAD_UP    0x01
#define DPAD_DOWN  0x02
#define DPAD_LEFT  0x08
#define DPAD_RIGHT 0x04

// Button bitmasks
#define BUTTON_A   0x0001
#define BUTTON_B   0x0002
#define BUTTON_X   0x0004
#define BUTTON_Y   0x0008
#define BUTTON_LB  0x0010
#define BUTTON_RB  0x0020
#define BUTTON_LS  0x0100
#define BUTTON_RS  0x0200

extern bool enableBrake;
extern bool isTurningLeft;
extern bool isTurningRight;
extern bool isSelfRotate;
extern bool enableOrientationPID;

void processGamepad(void);
