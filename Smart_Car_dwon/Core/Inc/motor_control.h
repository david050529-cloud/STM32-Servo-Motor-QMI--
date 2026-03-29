/* motor_control.h */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#define MOTOR1_PWM_CH   TIM_CHANNEL_4  // PE14
#define MOTOR1_PWM_CH2  TIM_CHANNEL_3  // PE13 (可能用于方向控制，实际取决于硬件)
#define MOTOR2_PWM_CH   TIM_CHANNEL_2  // PE11
#define MOTOR2_PWM_CH2  TIM_CHANNEL_1  // PE9

// 假设使用两路PWM控制电机正反转：一个通道为PWM，另一个为方向信号
// 这里简化：用一个通道控制速度，另一个通道控制方向（高电平正转，低电平反转）
void Motor_Init(void);
void Motor_SetSpeed(uint8_t motor_id, int16_t speed); // speed范围 -10000 ~ 10000，负号表示反转

#endif