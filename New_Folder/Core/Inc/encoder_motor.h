#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>

// 电机参数
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

struct EncoderMotorObject {
    // 编码器相关
    int64_t total_counter;     // 总计数值（含溢出）
    volatile int32_t overflow_num;      // 溢出次数
    int32_t ticks_overflow;    // 定时器溢出值（ARR）
    float tps;                 // 脉冲频率（Hz）
    float rps;                 // 转速（转/秒）

    // 硬件参数
    int32_t ticks_per_circle;  // 每转一圈的计数值

    // 硬件回调（设置PWM）
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);

    // 当前PWM值（用于调试）
    int current_pulse;
};

// 公共接口
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int32_t current_counter);
void motor_set_pulse(EncoderMotorObjectTypeDef *self, int pulse);  // 直接控制PWM

#endif