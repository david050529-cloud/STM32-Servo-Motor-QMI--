#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid,
              float kp, float ki, float kd,
              float out_max, float out_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = integral_max;

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void PID_SetTarget(PID_HandleTypeDef *pid, float target)
{
    pid->target = target;
}

// 增量式 PID：输出 Δu，调用者需维护上一次的输出值
float PID_Update(PID_HandleTypeDef *pid, float feedback)
{
    pid->feedback = feedback;
    float error = pid->target - feedback;

    float p_term = pid->kp * (error - pid->last_error);
    float i_term = pid->ki * error;
    float d_term = pid->kd * (error - pid->last_error);
    float delta = p_term + i_term + d_term;

    float new_output = pid->output + delta;

    // 抗积分饱和：只有当新输出在限幅范围内时才累加积分项
    if (new_output > pid->out_max) {
        new_output = pid->out_max;
        // 停止积分累加（不更新 integral）
    } else if (new_output < pid->out_min) {
        new_output = pid->out_min;
        // 停止积分累加
    } else {
        // 正常情况，允许积分累加
        if (pid->ki != 0.0f) {
            pid->integral += error;
            if (pid->integral > pid->integral_max)
                pid->integral = pid->integral_max;
            if (pid->integral < -pid->integral_max)
                pid->integral = -pid->integral_max;
        }
    }

    pid->output = new_output;
    pid->last_error = error;
    return pid->output;
}
// 位置式 PID（直接输出控制量）
float PID_Update_Position(PID_HandleTypeDef *pid, float feedback)
{
    pid->feedback = feedback;
    float error = pid->target - feedback;

    // 比例项
    float p_term = pid->kp * error;

    // 积分项（带限幅）
    pid->integral += error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    // 微分项
    float d_term = pid->kd * (error - pid->last_error);

    float output = p_term + i_term + d_term;
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;

    pid->output = output;
    pid->last_error = error;

    return output;
}