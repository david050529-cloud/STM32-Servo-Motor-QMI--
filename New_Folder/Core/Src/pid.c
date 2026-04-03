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
    self->integral += error * dt;
    if (self->integral > self->integral_limit)
        self->integral = self->integral_limit;
    else if (self->integral < -self->integral_limit)
        self->integral = -self->integral_limit;
    // 修正微分项符号
    float derivative = (error - self->prev_error) / dt;
    self->output = self->kp * error + self->ki * self->integral + self->kd * derivative;
    self->prev_error = error;
}