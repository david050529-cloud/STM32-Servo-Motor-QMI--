#include "encoder_motor.h"
#include <stdlib.h>

void encoder_update(EncoderMotorObjectTypeDef *self, float period, int32_t current_counter) {
    // 计算总计数（考虑溢出）
    int64_t new_total = (int64_t)self->overflow_num * (self->ticks_overflow + 1) + current_counter;
    int64_t delta = new_total - self->total_counter;
    self->total_counter = new_total;

    // 一阶低通滤波计算脉冲频率
    float tps_raw = (float)delta / period;
    self->tps = 0.9f * self->tps + 0.1f * tps_raw;

    // 计算转速（转/秒）
    self->rps = self->tps / self->ticks_per_circle;
}

void motor_set_pulse(EncoderMotorObjectTypeDef *self, int pulse) {
    // 限幅到PWM范围（-1000 ~ 1000）
    if (pulse > 1000) pulse = 1000;
    if (pulse < -1000) pulse = -1000;

    // 死区处理（PWM过小时电机不动）
    if (pulse > -50 && pulse < 50) pulse = 0;

    // 调用硬件回调
    self->set_pulse(self, pulse);
    self->current_pulse = pulse;
}

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self) {
    self->total_counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 60000;        // 与定时器ARR匹配
    self->ticks_per_circle = 3960;
    self->set_pulse = NULL;
}