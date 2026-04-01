#include "encoder_motor.h"
#include <stdlib.h>

void encoder_update(EncoderMotorObjectTypeDef *self, float period, uint32_t current_counter) {
    int64_t new_total;
    // 32位定时器（ARR=0xFFFFFFFF）直接使用当前计数值，忽略溢出（短时运行足够）
    if (self->ticks_overflow == 0xFFFFFFFF) {
        new_total = current_counter;
    } else {
        new_total = (int64_t)self->overflow_num * (self->ticks_overflow + 1) + current_counter;
    }
    int64_t delta = new_total - self->total_counter;
    self->total_counter = new_total;
    float tps_raw = (float)delta / period;
    self->tps = 0.9f * self->tps + 0.1f * tps_raw;
    self->rps = self->tps / self->ticks_per_circle;
}
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period) {
    // PID更新
    pid_controller_update(&self->pid_controller, self->rps, period);

    // PID输出直接作为PWM值（位置式）
    int pulse = (int)self->pid_controller.output;

    // 限幅
    if (pulse > 1000) pulse = 1000;
    if (pulse < -1000) pulse = -1000;

    // 死区处理（PWM过小时电机不动）


    // 输出
    self->set_pulse(self, pulse);
    self->current_pulse = pulse;
}

void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps) {
    if (rps > self->rps_limit) rps = self->rps_limit;
    if (rps < -self->rps_limit) rps = -self->rps_limit;
    self->pid_controller.set_point = rps;
}

void encoder_motor_object_init(EncoderMotorObjectTypeDef *self) {
    self->total_counter = 0;
    self->raw_counter = 0;
    self->overflow_num = 0;
    self->tps = 0;
    self->rps = 0;
    self->current_pulse = 0;
    self->ticks_overflow = 60000;        // 与定时器ARR匹配
    self->ticks_per_circle = 3960;
    self->rps_limit = 1.5f;
    pid_controller_init(&self->pid_controller, 0, 0, 0);
}
