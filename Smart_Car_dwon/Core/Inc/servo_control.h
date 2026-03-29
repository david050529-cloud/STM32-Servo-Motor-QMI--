/* servo_control.h */
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint8_t angle);  // 角度0~180

#endif