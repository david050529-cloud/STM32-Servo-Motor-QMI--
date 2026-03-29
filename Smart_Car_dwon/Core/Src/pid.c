/* pid.c */
#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float out_max, float out_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_max = out_max;
    pid->output_min = out_min;
}

float PID_Update(PID_HandleTypeDef *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    pid->integral += error;
    // 积分限幅
    if (pid->integral > pid->output_max / pid->Ki) pid->integral = pid->output_max / pid->Ki;
    if (pid->integral < pid->output_min / pid->Ki) pid->integral = pid->output_min / pid->Ki;

    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;

    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    return output;
}