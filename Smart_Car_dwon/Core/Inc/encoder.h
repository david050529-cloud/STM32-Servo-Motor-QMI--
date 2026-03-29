/* encoder.h */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

void Encoder_Init(void);
int16_t GetMotorSpeed(uint8_t motor_id);  // 返回速度，单位：转/分钟（rpm）或自定义

#endif