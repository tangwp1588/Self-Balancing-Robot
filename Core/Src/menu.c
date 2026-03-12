#include "menu.h"
#include "main.h"
#include "key.h"
//#include "OLED.h"
#include "st7789.h"
#include "pid.h"

uint8_t KeyNum;
int menu2;
float stepSize1 = 10.0f;
float stepSize2 = 0.1f;
float stepSize3 = 10.0f;
char MessageStepSize1[10], MessageKp1[10], MessageKi1[10], MessageKd1[10]; // for balancing PID
char MessageStepSize2[10], MessageEquilibrium2[10], MessageKp2[10], MessageKi2[10], MessageKd2[10]; // for speed PID
char MessageStepSize3[10], MessageKp3[10], MessageKi3[10], MessageKd3[10]; // for orientation PID

void mainMenu(void) {
	while(1) {
		menu2 = menu_lvl1();
		if (menu2 == 1) return;
		if (menu2 == 2) menu_lvl2_balancingPID();
		if (menu2 == 3) menu_lvl2_speedPID();
		if (menu2 == 4) menu_lvl2_orientationPID();
	}
}

void menu_lvl1_display(int LineNum) {
	// OLED_ShowString(0, 0, "<-             ", OLED_8X16);
	// OLED_ShowString(0, 16, "Option 1        ", OLED_8X16);
	// OLED_ShowString(0, 32, "Option 2        ", OLED_8X16);
	// OLED_ShowString(0, 48, "Option 3        ", OLED_8X16);
	//
	// if (line_no >= 1 && line_no <= 4) OLED_ReverseArea(0, (LineNum - 1) * 16, 64, 16);
	// OLED_Update();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2)
		ST7789_WriteString(0, SPACING, "Balancing PID", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0, SPACING, "Balancing PID", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 3)
		ST7789_WriteString(0, SPACING*2, "Speed PID     ", FONT, TXTCOLOUR, SELCOLOUR);
    else ST7789_WriteString(0, SPACING*2, "Speed PID     ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 4)
		ST7789_WriteString(0, SPACING*3, "Orientation PID", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0, SPACING*3, "Orientation PID", FONT, TXTCOLOUR, BGCOLOUR);
}

int menu_lvl1(void) {
	uint8_t flag = 1;
	menu_lvl1_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			flag = 1;
			ST7789_Fill_Color(BGCOLOUR);
			return flag;
		}
		if(KeyNum == 3) {
			flag++;
			if(flag == 5) flag = 1;
		}
		if(KeyNum == 2) {
			flag--;
			if(flag == 0) flag = 4;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			ST7789_Fill_Color(BGCOLOUR);
			return flag;
		}

		menu_lvl1_display(flag);
	}
}

void balancingPID_printf() {
	sprintf(MessageStepSize1, "%9.4f", stepSize1);
	sprintf(MessageKp1, "%9.4f", pidBalancingConstant.Kp);
	sprintf(MessageKi1, "%9.4f", pidBalancingConstant.Ki);
	sprintf(MessageKd1, "%9.4f", pidBalancingConstant.Kd);
}

void speedPID_printf() {
	sprintf(MessageStepSize2, "%9.4f", stepSize2);
	sprintf(MessageEquilibrium2, "%9.4f", Equilibrium_Angle);
	sprintf(MessageKp2, "%9.4f", pidSpeedConstant.Kp);
	sprintf(MessageKi2, "%9.4f", pidSpeedConstant.Ki);
	sprintf(MessageKd2, "%9.4f", pidSpeedConstant.Kd);
}

void orientationPID_printf() {
	sprintf(MessageStepSize3, "%9.4f", stepSize3);
	sprintf(MessageKp3, "%9.4f", pidOrientationConstant.Kp);
	sprintf(MessageKi3, "%9.4f", pidOrientationConstant.Ki);
	sprintf(MessageKd3, "%9.4f", pidOrientationConstant.Kd);
}

void menu_lvl2_balancingPID_display(int LineNum) {
	balancingPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl2_balancingPID(void) {
	uint8_t flag_lvl2 = 1, flag_lvl3 = 0;
	menu_lvl2_balancingPID_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = 1;
		}
		if(KeyNum == 3) {
			flag_lvl2++;
			if(flag_lvl2 == 6) flag_lvl2 = 1;
		}
		if(KeyNum == 2) {
			flag_lvl2--;
			if(flag_lvl2 == 0) flag_lvl2 = 5;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = flag_lvl2;
		}

		if(flag_lvl3 == 1) {
			ST7789_Fill_Color(BGCOLOUR);
			return 0;
		}
		if(flag_lvl3 == 2) flag_lvl3 = menu_lvl3_balancingPID_stepSize();
		if(flag_lvl3 == 3) flag_lvl3 = menu_lvl3_balancingPID_constants(1);
		if(flag_lvl3 == 4) flag_lvl3 = menu_lvl3_balancingPID_constants(2);
		if(flag_lvl3 == 5) flag_lvl3 = menu_lvl3_balancingPID_constants(3);

		menu_lvl2_balancingPID_display(flag_lvl2);
	}
}

void menu_lvl2_speedPID_display(int LineNum) {
	speedPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 6) {
		ST7789_WriteString(0, SPACING*5, "Eqm. :", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*5, MessageEquilibrium2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*5, "Eqm. :", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*5, MessageEquilibrium2, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl2_speedPID(void) {
	uint8_t flag_lvl2 = 1, flag_lvl3 = 0;
	menu_lvl2_speedPID_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = 1;
		}
		if(KeyNum == 3) {
			flag_lvl2++;
			if(flag_lvl2 == 7) flag_lvl2 = 1;
		}
		if(KeyNum == 2) {
			flag_lvl2--;
			if(flag_lvl2 == 0) flag_lvl2 = 6;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = flag_lvl2;
		}

		if(flag_lvl3 == 1) {
			ST7789_Fill_Color(BGCOLOUR);
			return 0;
		}
		if(flag_lvl3 == 2) flag_lvl3 = menu_lvl3_speedPID_stepSize();
		if(flag_lvl3 == 3) flag_lvl3 = menu_lvl3_speedPID_constants(1);
		if(flag_lvl3 == 4) flag_lvl3 = menu_lvl3_speedPID_constants(2);
		if(flag_lvl3 == 5) flag_lvl3 = menu_lvl3_speedPID_constants(3);
		if(flag_lvl3 == 6) flag_lvl3 = menu_lvl3_speedPID_constants(4);

		menu_lvl2_speedPID_display(flag_lvl2);
	}
}

void menu_lvl2_orientationPID_display(int LineNum) {
	orientationPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize3, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp3, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi3, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd3, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd3, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl2_orientationPID(void) {
	uint8_t flag_lvl2 = 1, flag_lvl3 = 0;
	menu_lvl2_orientationPID_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = 1;
		}
		if(KeyNum == 3) {
			flag_lvl2++;
			if(flag_lvl2 == 6) flag_lvl2 = 1;
		}
		if(KeyNum == 2) {
			flag_lvl2--;
			if(flag_lvl2 == 0) flag_lvl2 = 5;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = flag_lvl2;
		}

		if(flag_lvl3 == 1) {
			ST7789_Fill_Color(BGCOLOUR);
			return 0;
		}
		if(flag_lvl3 == 2) flag_lvl3 = menu_lvl3_orientationPID_stepSize();
		if(flag_lvl3 == 3) flag_lvl3 = menu_lvl3_orientationPID_constants(1);
		if(flag_lvl3 == 4) flag_lvl3 = menu_lvl3_orientationPID_constants(2);
		if(flag_lvl3 == 5) flag_lvl3 = menu_lvl3_orientationPID_constants(3);

		menu_lvl2_orientationPID_display(flag_lvl2);
	}
}

// lvl3 - balancing PID constant adjust
void menu_lvl3_balancingPID_display(int LineNum) {
	balancingPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl3_balancingPID_stepSize() {
	while (1) {
		menu_lvl3_balancingPID_display(2); // Step size option is in line 2

		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			stepSize1 *= 10;
			if (stepSize1 > 1000) stepSize1 = 1;
		}
		if(KeyNum == 3) {
			stepSize1 /= 10;
			if (stepSize1 <= 0.00001) stepSize1 = 1;
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
	}
}

int menu_lvl3_balancingPID_constants(int id) {
	while (1) {
		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidBalancingConstant.integral = 0;
				pidBalancingConstant.prev_error = 0;
				if (id == 1) pidBalancingConstant.Kp -= stepSize1;
				if (id == 2) pidBalancingConstant.Ki -= stepSize1;
				if (id == 3) pidBalancingConstant.Kd -= stepSize1;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		if(KeyNum == 3) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidBalancingConstant.integral = 0;
				pidBalancingConstant.prev_error = 0;
				if (id == 1) pidBalancingConstant.Kp += stepSize1;
				if (id == 2) pidBalancingConstant.Ki += stepSize1;
				if (id == 3) pidBalancingConstant.Kd += stepSize1;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
		switch(id) {
			case 1:{
				menu_lvl3_balancingPID_display(3); // Kp option is in line 3
				break;
			}
			case 2:{
				menu_lvl3_balancingPID_display(4); // Ki option is in line 4
				break;
			}
			case 3: {
				menu_lvl3_balancingPID_display(5); // Kd option is in line 5
				break;
			}
		}
	}
}

// lvl3 - speed PID constant adjust
void menu_lvl3_speedPID_display(int LineNum) {
	speedPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 6) {
		ST7789_WriteString(0, SPACING*5, "Eqm. :", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*5, MessageEquilibrium2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*5, "Eqm. :", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*5, MessageEquilibrium2, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl3_speedPID_stepSize() {
	while (1) {
		menu_lvl3_speedPID_display(2); // Step size option is in line 2

		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			stepSize2 *= 10;
			if (stepSize2 > 1000) stepSize2 = 1;
		}
		if(KeyNum == 3) {
			stepSize2 /= 10;
			if (stepSize2 <= 0.00001) stepSize2 = 1;
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
	}
}

int menu_lvl3_speedPID_constants(int id) {
	while (1) {
		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidSpeedConstant.integral = 0;
				pidSpeedConstant.prev_error = 0;
				if (id == 1) {
					pidSpeedConstant.Kp -= stepSize2;
					// pidSpeedConstant.Ki = pidSpeedConstant.Kp / 200;
				}
				if (id == 2) pidSpeedConstant.Ki -= stepSize2;
				if (id == 3) pidSpeedConstant.Kd -= stepSize2;
				if (id == 4) Equilibrium_Angle -= stepSize2;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		if(KeyNum == 3) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidSpeedConstant.integral = 0;
				pidSpeedConstant.prev_error = 0;
				if (id == 1) {
					pidSpeedConstant.Kp += stepSize2;
					// pidSpeedConstant.Ki = pidSpeedConstant.Kp / 200;
				}
				if (id == 2) pidSpeedConstant.Ki += stepSize2;
				if (id == 3) pidSpeedConstant.Kd += stepSize2;
				if (id == 4) Equilibrium_Angle += stepSize2;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
		switch(id) {
			case 1:{
				menu_lvl3_speedPID_display(3); // Kp option is in line 3
				break;
			}
			case 2:{
				menu_lvl3_speedPID_display(4); // Ki option is in line 4
				break;
			}
			case 3: {
				menu_lvl3_speedPID_display(5); // Kd option is in line 5
				break;
			}
			case 4: {
				menu_lvl3_speedPID_display(6); // Eqm. option is in line 5
				break;
			}
		}
	}
}

// lvl3 - orientation PID constant adjust
void menu_lvl3_orientationPID_display(int LineNum) {
	orientationPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize3, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp3, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi3, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi3, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd3, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd3, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl3_orientationPID_stepSize() {
	while (1) {
		menu_lvl3_orientationPID_display(2); // Step size option is in line 2

		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			stepSize3 *= 10;
			if (stepSize3 > 1000) stepSize3 = 1;
		}
		if(KeyNum == 3) {
			stepSize3 /= 10;
			if (stepSize3 <= 0.00001) stepSize3 = 1;
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
	}
}

int menu_lvl3_orientationPID_constants(int id) {
	while (1) {
		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidOrientationConstant.integral = 0;
				pidOrientationConstant.prev_error = 0;
				if (id == 1) pidOrientationConstant.Kp -= stepSize3;
				if (id == 2) pidOrientationConstant.Ki -= stepSize3;
				if (id == 3) pidOrientationConstant.Kd -= stepSize3;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		if(KeyNum == 3) {
			if(osMutexAcquire(PIDValueMutexHandle, 2) == osOK) {
				pidOrientationConstant.integral = 0;
				pidOrientationConstant.prev_error = 0;
				if (id == 1) pidOrientationConstant.Kp += stepSize3;
				if (id == 2) pidOrientationConstant.Ki += stepSize3;
				if (id == 3) pidOrientationConstant.Kd += stepSize3;
				osMutexRelease(PIDValueMutexHandle);
			}
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
		switch(id) {
			case 1:{
				menu_lvl3_orientationPID_display(3); // Kp option is in line 3
				break;
			}
			case 2:{
				menu_lvl3_orientationPID_display(4); // Ki option is in line 4
				break;
			}
			case 3: {
				menu_lvl3_orientationPID_display(5); // Kd option is in line 5
				break;
			}
		}
	}
}