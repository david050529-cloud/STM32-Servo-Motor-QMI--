#ifndef __MOTOR_PORTING_H__
#define __MOTOR_PORTING_H__

#include "encoder_motor.h"

extern EncoderMotorObjectTypeDef motor1;
extern EncoderMotorObjectTypeDef motor2;

extern volatile float imu_roll, imu_pitch, imu_yaw;

void motor_init(void);
// 新增：设置舵机角度 (0-180)
void servo_set_angle(uint16_t angle);

#endif