#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

// 电机型号参数（示例）
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_JGB520_PID_KP   63.0f
#define MOTOR_JGB520_PID_KI   2.6f
#define MOTOR_JGB520_PID_KD   2.4f
#define MOTOR_JGB520_RPS_LIMIT 1.5f

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

struct EncoderMotorObject {
    // 状态变量
    int64_t raw_counter;       // 原始计数值（不含溢出）
    int64_t total_counter;     // 总计数值（含溢出）
    int32_t overflow_num;      // 溢出次数
    int32_t ticks_overflow;    // 定时器溢出值（ARR）
    float tps;                 // 脉冲频率（Hz）
    float rps;                 // 转速（转/秒）
    int current_pulse;         // 当前PWM值

    // PID控制器
    PID_ControllerTypeDef pid_controller;

    // 硬件参数
    int32_t ticks_per_circle;  // 每转一圈的计数值
    float rps_limit;           // 转速限制

    // 硬件回调
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};

// 公共接口
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);
void encoder_update(EncoderMotorObjectTypeDef *self, float period, uint32_t current_counter);
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif