#ifndef _PID_H
#define _PID_H

#include <stdint.h>

typedef struct {
    float set_point;      // 目标值
    float kp;             // 比例增益
    float ki;             // 积分增益
    float kd;             // 微分增益

    float integral;       // 积分累加和
    float prev_error;     // 上一次误差
    float output;         // PID输出

    float integral_limit; // 积分限幅（可选）
} PID_ControllerTypeDef;

void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);
void pid_controller_update(PID_ControllerTypeDef *self, float actual, float dt);

#endif