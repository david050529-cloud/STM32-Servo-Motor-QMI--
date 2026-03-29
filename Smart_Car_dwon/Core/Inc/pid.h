/* pid.h */
#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_max;
    float output_min;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float out_max, float out_min);
float PID_Update(PID_HandleTypeDef *pid, float setpoint, float measurement);

#endif