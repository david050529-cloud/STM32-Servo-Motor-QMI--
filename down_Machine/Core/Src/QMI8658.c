#include "QMI8658.h"
#include "i2c.h"    // 使用 hi2c2
#include "main.h"   // 使用 HAL_Delay
#include <math.h>

// 内部缩放因子（根据量程计算）
static float acc_scale = 1.0f;   // LSB/g
static float gyro_scale = 1.0f;  // LSB/(dps)

// 上一次读取的时间戳（用于姿态解算）
static uint32_t last_tick = 0;

// 互补滤波姿态角（静态变量）
static float roll_angle = 0.0f;
static float pitch_angle = 0.0f;
static float yaw_angle = 0.0f;

// I2C 读写辅助函数
static uint8_t QMI8658_ReadReg(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c2, QMI8658_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

static void QMI8658_WriteReg(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c2, QMI8658_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

static int16_t QMI8658_ReadWord(uint8_t reg) {
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c2, QMI8658_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

// 配置加速度计
static void QMI8658_ConfigAcc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr) {
    uint8_t reg_val = (uint8_t)range | (uint8_t)odr;
    QMI8658_WriteReg(Qmi8658Register_Ctrl2, reg_val);
    // 根据量程设置缩放因子 (LSB/g)
    switch (range) {
        case Qmi8658AccRange_2g:  acc_scale = 16384.0f; break;
        case Qmi8658AccRange_4g:  acc_scale = 8192.0f;  break;
        case Qmi8658AccRange_8g:  acc_scale = 4096.0f;  break;
        case Qmi8658AccRange_16g: acc_scale = 2048.0f;  break;
        default:                  acc_scale = 4096.0f;  break;
    }
}

// 配置陀螺仪
static void QMI8658_ConfigGyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr) {
    uint8_t reg_val = (uint8_t)range | (uint8_t)odr;
    QMI8658_WriteReg(Qmi8658Register_Ctrl3, reg_val);
    // 根据量程设置缩放因子 (LSB/(dps))
    switch (range) {
        case Qmi8658GyrRange_16dps:   gyro_scale = 2048.0f; break;
        case Qmi8658GyrRange_32dps:   gyro_scale = 1024.0f; break;
        case Qmi8658GyrRange_64dps:   gyro_scale = 512.0f;  break;
        case Qmi8658GyrRange_128dps:  gyro_scale = 256.0f;  break;
        case Qmi8658GyrRange_256dps:  gyro_scale = 128.0f;  break;
        case Qmi8658GyrRange_512dps:  gyro_scale = 64.0f;   break;
        case Qmi8658GyrRange_1024dps: gyro_scale = 32.0f;   break;
        case Qmi8658GyrRange_2048dps: gyro_scale = 16.0f;   break;
        default:                      gyro_scale = 64.0f;   break;
    }
}

// 使能传感器
static void QMI8658_EnableSensors(uint8_t enable) {
    // Ctrl7: bit0=ACC, bit1=GYRO
    QMI8658_WriteReg(Qmi8658Register_Ctrl7, enable);
    HAL_Delay(1);
}

// 软复位并执行片上校准
static void QMI8658_SoftResetAndCalibrate(void) {
    QMI8658_WriteReg(Qmi8658Register_Reset, 0xB0);  // 软复位
    HAL_Delay(10);
    // 发送片上校准命令
    QMI8658_WriteReg(Qmi8658Register_Ctrl9, 0xA2);  // On-Demand Calibration
    HAL_Delay(2200);  // 等待校准完成（约2秒）
    QMI8658_WriteReg(Qmi8658Register_Ctrl9, 0x00);
    HAL_Delay(100);
}

// 初始化函数
bool QMI8658_Init(void) {
    // 1. 检查芯片ID
    uint8_t whoami = QMI8658_ReadReg(Qmi8658Register_WhoAmI);
    if (whoami != 0x05) {
        return false;
    }

    // 2. 软复位并校准
    QMI8658_SoftResetAndCalibrate();

    // 3. 配置Ctrl1：中断映射（不使用中断时可保持默认）
    QMI8658_WriteReg(Qmi8658Register_Ctrl1, 0x60); // 使能内部时钟，不映射中断

    // 4. 配置加速度计（量程±8g，ODR 250Hz）
    QMI8658_ConfigAcc(Qmi8658AccRange_8g, Qmi8658AccOdr_250Hz);

    // 5. 配置陀螺仪（量程±512dps，ODR 250Hz）
    QMI8658_ConfigGyro(Qmi8658GyrRange_512dps, Qmi8658GyrOdr_250Hz);

    // 6. 使能加速度计和陀螺仪
    QMI8658_EnableSensors(QMI8658_ACCGYR_ENABLE);

    // 7. 可选：关闭低通滤波（Ctrl5）
    QMI8658_WriteReg(Qmi8658Register_Ctrl5, 0x00);

    last_tick = HAL_GetTick();
    return true;
}

// 读取原始数据（加速度单位g，角速度单位dps）
void QMI8658_ReadRaw(float acc[3], float gyro[3]) {
    // 读取加速度原始值
    int16_t ax_raw = QMI8658_ReadWord(Qmi8658Register_Ax_L);
    int16_t ay_raw = QMI8658_ReadWord(Qmi8658Register_Ay_L);
    int16_t az_raw = QMI8658_ReadWord(Qmi8658Register_Az_L);
    // 读取陀螺仪原始值
    int16_t gx_raw = QMI8658_ReadWord(Qmi8658Register_Gx_L);
    int16_t gy_raw = QMI8658_ReadWord(Qmi8658Register_Gy_L);
    int16_t gz_raw = QMI8658_ReadWord(Qmi8658Register_Gz_L);

    // 转换为物理单位
    acc[0] = (float)ax_raw / acc_scale;
    acc[1] = (float)ay_raw / acc_scale;
    acc[2] = (float)az_raw / acc_scale;
    gyro[0] = (float)gx_raw / gyro_scale;
    gyro[1] = (float)gy_raw / gyro_scale;
    gyro[2] = (float)gz_raw / gyro_scale;
}

// 读取温度（摄氏度）
float QMI8658_ReadTemperature(void) {
    int16_t temp_raw = QMI8658_ReadWord(Qmi8658Register_Temp_L);
    return (float)temp_raw / 256.0f + 25.0f;  // 公式参考数据手册
}

// 互补滤波姿态解算（需要周期性调用，dt为采样间隔秒）
void QMI8658_GetEuler(float dt, float *roll, float *pitch, float *yaw) {
    float acc[3], gyro[3];
    QMI8658_ReadRaw(acc, gyro);

    // 由加速度计算俯仰和横滚（单位：度）
    float acc_pitch = atan2f(-acc[0], sqrtf(acc[1]*acc[1] + acc[2]*acc[2])) * 57.29578f;
    float acc_roll  = atan2f(acc[1], acc[2]) * 57.29578f;

    // 互补滤波
    float tau = 0.98f;  // 互补滤波系数
    pitch_angle = tau * (pitch_angle + gyro[1] * dt) + (1.0f - tau) * acc_pitch;
    roll_angle  = tau * (roll_angle  + gyro[0] * dt) + (1.0f - tau) * acc_roll;
    // 偏航角由陀螺仪积分（无磁力计辅助会漂移）
    yaw_angle += gyro[2] * dt;

    *roll = roll_angle;
    *pitch = pitch_angle;
    *yaw = yaw_angle;
}

// 简便版：自动计算时间差（需要在主循环中周期性调用，建议固定频率）
void QMI8658_GetEulerSimple(float *roll, float *pitch, float *yaw) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;
    last_tick = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // 限制范围
    QMI8658_GetEuler(dt, roll, pitch, yaw);
}