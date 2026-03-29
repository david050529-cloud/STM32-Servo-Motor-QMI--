/* servo_control.c */
#include "servo_control.h"
#include "tim.h"
#include "gpio.h"

// 舵机1使用TIM3_CH3 (PC8)
#define SERVO1_TIM   &htim3
#define SERVO1_CH    TIM_CHANNEL_3
#define SERVO1_MIN_PULSE  500   // 0度对应500us
#define SERVO1_MAX_PULSE  2500  // 180度对应2500us
#define SERVO1_PERIOD_MS   20    // 20ms周期

// 舵机2使用GPIO模拟PWM (PA12)
#define SERVO2_PIN   PWM_SERVO_2_Pin
#define SERVO2_PORT  PWM_SERVO_2_GPIO_Port

static void SetServo1Angle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint32_t pulse_us = SERVO1_MIN_PULSE + (SERVO1_MAX_PULSE - SERVO1_MIN_PULSE) * angle / 180;
    // 假设定时器时钟频率为84MHz，预分频器84-1，计数器频率1MHz，周期20ms对应20000计数值
    // 实际需要根据TIM3的配置计算：htim3.Init.Prescaler和Period
    // 这里假设已经配置好定时器周期为20000（20ms），占空比 = pulse_us / 20000 * 100%
    uint32_t pulse = (uint32_t)(pulse_us * 20000 / 20000);  // 实际需要根据定时器计数周期调整
    // 更简单：直接使用HAL库函数设置占空比
    __HAL_TIM_SET_COMPARE(SERVO1_TIM, SERVO1_CH, pulse);
}

static void SetServo2Angle(uint8_t angle)
{
    // 软件模拟PWM，需要定时器中断或任务周期翻转引脚
    // 这里简单起见，使用一个软定时器，在单独的任务中处理
    // 实际可以使用另一个硬件定时器，或者直接用GPIO输出PWM
    // 这里暂不实现，留作扩展
}

void Servo_Init(void)
{
    // 启动TIM3的PWM输出
    HAL_TIM_PWM_Start(SERVO1_TIM, SERVO1_CH);
    // 初始化舵机2的GPIO为输出
    HAL_GPIO_WritePin(SERVO2_PORT, SERVO2_PIN, GPIO_PIN_RESET);
}

void Servo_SetAngle(uint8_t servo_id, uint8_t angle)
{
    if (servo_id == 1) {
        SetServo1Angle(angle);
    } else if (servo_id == 2) {
        SetServo2Angle(angle);
    }
}