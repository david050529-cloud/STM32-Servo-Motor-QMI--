#include "pid.h"

void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd) {
    self->set_point = 0;
    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->integral = 0;
    self->prev_error = 0;
    self->output = 0;
    self->integral_limit = 1000.0f; // 可根据实际调整
}

void pid_controller_update(PID_ControllerTypeDef *self, float actual, float dt) {
    float error = self->set_point - actual;

    // 积分累加并限幅
    self->integral += error * dt;
    if (self->integral > self->integral_limit)
        self->integral = self->integral_limit;
    else if (self->integral < -self->integral_limit)
        self->integral = -self->integral_limit;

    // 微分（实际值微分，避免目标值突变）
    float derivative = (self->prev_error - error) / dt; // 注意符号：常用 (error - prev_error)/dt，这里使用负反馈

    // 位置式PID输出
    self->output = self->kp * error + self->ki * self->integral + self->kd * derivative;

    // 保存误差
    self->prev_error = error;
}