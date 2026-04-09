#include "tim.h"
#include "encoder_motor.h"
#include "motor_porting.h"
#include "pid.h"

// 全局电机对象
EncoderMotorObjectTypeDef motor1;
EncoderMotorObjectTypeDef motor2;

PID_HandleTypeDef pid_motor1;
PID_HandleTypeDef pid_motor2;
// 外部定时器句柄
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

// 电机1 PWM 设置 (TIM1 CH3/CH4)
void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int pulse) {
    if (pulse > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
    } else if (pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

// 电机2 PWM 设置 (TIM1 CH1/CH2)
void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int pulse) {
    if (pulse > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    } else if (pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

// 电机初始化（两个电机）
void motor_init(void) {
    // ========== 电机1 初始化 ==========
    encoder_motor_object_init(&motor1);
    motor1.ticks_overflow = 60000;                     // TIM5 ARR
    motor1.ticks_per_circle = MOTOR_JGB520_TICKS_PER_CIRCLE;
    motor1.set_pulse = motor1_set_pulse;

    // ========== 电机2 初始化 ==========
    encoder_motor_object_init(&motor2);
    motor2.ticks_overflow = 60000;                     // TIM2 ARR
    motor2.ticks_per_circle = MOTOR_JGB520_TICKS_PER_CIRCLE;
    motor2.set_pulse = motor2_set_pulse;

    // 启动 TIM1 所有 PWM 通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // ========== 配置 TIM5 编码器（电机1） ==========
    HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_AUTORELOAD(&htim5, motor1.ticks_overflow);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    TIM5->CR1 |= TIM_CR1_ARPE;
    TIM5->DIER |= TIM_DIER_UIE;          // 使能更新中断（用于溢出计数）
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    // ========== 配置 TIM2 编码器（电机2） ==========
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_AUTORELOAD(&htim2, motor2.ticks_overflow);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->DIER |= TIM_DIER_UIE;          // 使能更新中断
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // 可选：启动TIM6用于周期性读取编码器（如果需要监控转速）
    HAL_TIM_Base_Start_IT(&htim6);

    // 初始化 PID 参数（示例，需根据实际调参）
    //对应PID_Update
    // PID_Init(&pid_motor1, 8.0f, 0.05f, 0.05f, 1000.0f, -1000.0f, 500.0f);
    // PID_Init(&pid_motor2, 8.0f, 0.05f, 0.05f, 1000.0f, -1000.0f, 500.0f);

    //对应PID_Update_Position
    PID_Init(&pid_motor1, 10.0f, 0.1f, 0.5f, 1000.0f, -1000.0f, 500.0f);
    PID_Init(&pid_motor2, 10.0f, 0.1f, 0.5f, 1000.0f, -1000.0f, 500.0f);
}

void start_encoder_periodic_update(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}
