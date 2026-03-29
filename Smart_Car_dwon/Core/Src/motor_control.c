/* motor_control.c */
#include "motor_control.h"
#include "tim.h"

static void setMotor1PWM(int16_t pwm_value)
{
    // 限制pwm范围0~10000，映射到定时器自动重载值（ARR=65535）
    if (pwm_value > 10000) pwm_value = 10000;
    if (pwm_value < -10000) pwm_value = -10000;
    uint32_t pulse = (uint32_t)((pwm_value > 0 ? pwm_value : -pwm_value) * 65535 / 10000);
    if (pulse > 65535) pulse = 65535;

    // 根据符号设置方向引脚（假设高电平正转，低电平反转）
    // 注意：需要配置对应的GPIO为输出，这里假设使用PE14和PE13的PWM，而方向由另一个引脚控制？
    // 根据现有引脚配置，PE14和PE13分别连接电机1的CH1和CH2，可能需要互补PWM输出。
    // 这里简化：如果硬件是H桥，一个通道PWM，另一个通道直接控制方向。
    // 但实际引脚配置中，Motor1_CH1是PE14，Motor1_CH2是PE13，都配置为TIM1的PWM输出。
    // 因此可能需要同时设置两个通道的占空比来实现正反转（如差速控制）。
    // 这里暂时只用一个通道，另一个通道固定为0或1。
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR1_PWM_CH, pulse);
    if (pwm_value > 0) {
        // 正转
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR1_PWM_CH2, 0);
    } else {
        // 反转
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR1_PWM_CH2, 65535);
    }
}

static void setMotor2PWM(int16_t pwm_value)
{
    if (pwm_value > 10000) pwm_value = 10000;
    if (pwm_value < -10000) pwm_value = -10000;
    uint32_t pulse = (uint32_t)((pwm_value > 0 ? pwm_value : -pwm_value) * 65535 / 10000);
    if (pulse > 65535) pulse = 65535;

    __HAL_TIM_SET_COMPARE(&htim1, MOTOR2_PWM_CH, pulse);
    if (pwm_value > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR2_PWM_CH2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR2_PWM_CH2, 65535);
    }
}

void Motor_Init(void)
{
    // 启动TIM1的PWM输出
    HAL_TIM_PWM_Start(&htim1, MOTOR1_PWM_CH);
    HAL_TIM_PWM_Start(&htim1, MOTOR1_PWM_CH2);
    HAL_TIM_PWM_Start(&htim1, MOTOR2_PWM_CH);
    HAL_TIM_PWM_Start(&htim1, MOTOR2_PWM_CH2);
}

void Motor_SetSpeed(uint8_t motor_id, int16_t speed)
{
    if (motor_id == 1) {
        setMotor1PWM(speed);
    } else if (motor_id == 2) {
        setMotor2PWM(speed);
    }
}