#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

// PID 结构体（增量式）
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数

    float out_max;          // 输出上限
    float out_min;          // 输出下限
    float integral_max;     // 积分限幅

    float target;           // 目标值
    float feedback;         // 反馈值

    float last_error;       // 上一次误差
    float integral;         // 积分累积
    float output;           // PID 输出
} PID_HandleTypeDef;

// 初始化 PID 参数
void PID_Init(PID_HandleTypeDef *pid,
              float kp, float ki, float kd,
              float out_max, float out_min, float integral_max);

// 重置 PID 状态
void PID_Reset(PID_HandleTypeDef *pid);

// 设置目标值
void PID_SetTarget(PID_HandleTypeDef *pid, float target);

// 增量式 PID 计算，返回输出值（PWM 增量，需要累加到上次输出）
float PID_Update(PID_HandleTypeDef *pid, float feedback);

// 位置式 PID 计算（可选）
float PID_Update_Position(PID_HandleTypeDef *pid, float feedback);

#endif // __PID_H__