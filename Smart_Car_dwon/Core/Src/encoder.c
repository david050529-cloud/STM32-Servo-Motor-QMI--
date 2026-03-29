/* encoder.c */
#include "encoder.h"
#include "tim.h"

// 假设电机编码器每转产生N个脉冲，这里需要根据实际编码器线数设置
#define ENCODER_PPR  4000   // 编码器每转脉冲数（4倍频后）
#define SAMPLE_TIME_MS 10   // 速度采样周期（ms）

static int32_t last_counter[2] = {0};
static int16_t current_speed[2] = {0};

void Encoder_Init(void)
{
    // 启动编码器定时器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 电机2编码器
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);  // 电机1编码器
    // 读取初始计数值
    last_counter[0] = __HAL_TIM_GET_COUNTER(&htim5);   // 电机1
    last_counter[1] = __HAL_TIM_GET_COUNTER(&htim2);   // 电机2
}

void Update_Speed(void)
{
    // 此函数应在固定时间间隔调用（例如10ms）
    int32_t counter[2];
    counter[0] = __HAL_TIM_GET_COUNTER(&htim5);
    counter[1] = __HAL_TIM_GET_COUNTER(&htim2);

    int32_t delta[2];
    delta[0] = counter[0] - last_counter[0];
    delta[1] = counter[1] - last_counter[1];
    last_counter[0] = counter[0];
    last_counter[1] = counter[1];

    // 计算转速：每秒脉冲数 = delta / (SAMPLE_TIME_MS/1000) = delta * 1000 / SAMPLE_TIME_MS
    // 转速(rpm) = (每秒脉冲数) * 60 / 编码器线数
    for (int i=0; i<2; i++) {
        int32_t pulses_per_sec = delta[i] * 1000 / SAMPLE_TIME_MS;
        current_speed[i] = (int16_t)(pulses_per_sec * 60 / ENCODER_PPR);
        if (current_speed[i] > 32767) current_speed[i] = 32767;
        if (current_speed[i] < -32768) current_speed[i] = -32768;
    }
}

int16_t GetMotorSpeed(uint8_t motor_id)
{
    if (motor_id == 1) return current_speed[0];
    if (motor_id == 2) return current_speed[1];
    return 0;
}