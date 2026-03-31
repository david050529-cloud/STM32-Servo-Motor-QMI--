#include "tim.h"
#include "encoder_motor.h"
#include "motor_porting.h"
#include "QMI8658.h"

// 全局电机对象
EncoderMotorObjectTypeDef motor1;
EncoderMotorObjectTypeDef motor2;

volatile float imu_roll = 0, imu_pitch = 0, imu_yaw = 0;

// 舵机PWM控制：TIM3_CH3 (PC8)
void servo_set_angle(uint16_t angle) {
    // if (angle > 180) angle = 180;
    // 假设舵机对应0度时PWM高电平时间0.5ms，180度时2.5ms，周期20ms
    // 定时器3频率：系统时钟168MHz，APB1=42MHz，Prescaler=999 → 42kHz，Period=3359 → 约20ms周期
    // CCR值范围：0.5ms对应 0.5/20 * 3360 ≈ 84；2.5ms对应 2.5/20*3360 ≈ 420
    // uint32_t ccr = 84 + (angle * (420 - 84) / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angle);
}

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
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
    } else if (pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

// 电机初始化（两个电机）
void motor_init(void) {
    // ========== 电机1 初始化 ==========
    encoder_motor_object_init(&motor1);
    motor1.ticks_overflow = 0xFFFFFFFF;   // 32位定时器，无需溢出处理
    motor1.ticks_per_circle = MOTOR_JGB520_TICKS_PER_CIRCLE;
    motor1.rps_limit = MOTOR_JGB520_RPS_LIMIT;
    // 设置PID参数
    motor1.pid_controller.kp = MOTOR_JGB520_PID_KP;
    motor1.pid_controller.ki = MOTOR_JGB520_PID_KI;
    motor1.pid_controller.kd = MOTOR_JGB520_PID_KD;
    motor1.set_pulse = motor1_set_pulse;

    // ========== 电机2 初始化 ==========
    encoder_motor_object_init(&motor2);
    motor2.ticks_overflow = 0xFFFFFFFF;
    motor2.ticks_per_circle = MOTOR_JGB520_TICKS_PER_CIRCLE;
    motor2.rps_limit = MOTOR_JGB520_RPS_LIMIT;
    motor2.pid_controller.kp = MOTOR_JGB520_PID_KP;
    motor2.pid_controller.ki = MOTOR_JGB520_PID_KI;
    motor2.pid_controller.kd = MOTOR_JGB520_PID_KD;
    motor2.set_pulse = motor2_set_pulse;

    // 启动 TIM1 所有 PWM 通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 启动 TIM3 PWM 用于舵机
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    servo_set_angle(90);  // 初始角度90度

    // ========== 配置 TIM5 编码器（电机1） ==========
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    // 设置计数器初始值
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    // 启用更新中断（可选，但32位模式可不用，这里不启用）
    // 直接读取计数器值即可

    // ========== 配置 TIM2 编码器（电机2） ==========
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 启动TIM6作为控制周期定时器（10ms中断）
    HAL_TIM_Base_Start_IT(&htim6);
}