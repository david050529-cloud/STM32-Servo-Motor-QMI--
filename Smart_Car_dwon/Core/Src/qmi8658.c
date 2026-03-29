/* qmi8658.c */
#include "qmi8658.h"
#include "i2c.h"

// QMI8658寄存器地址
#define QMI8658_WHO_AM_I      0x00
#define QMI8658_REVISION_ID   0x01
#define QMI8658_CTRL1         0x02
#define QMI8658_CTRL2         0x03
#define QMI8658_CTRL3         0x04
#define QMI8658_CTRL4         0x05
#define QMI8658_CTRL5         0x06
#define QMI8658_CTRL6         0x07
#define QMI8658_CTRL7         0x08
#define QMI8658_CTRL8         0x09
#define QMI8658_CTRL9         0x0A
#define QMI8658_ACCEL_X_L     0x35
#define QMI8658_ACCEL_X_H     0x36
#define QMI8658_ACCEL_Y_L     0x37
#define QMI8658_ACCEL_Y_H     0x38
#define QMI8658_ACCEL_Z_L     0x39
#define QMI8658_ACCEL_Z_H     0x3A
#define QMI8658_GYRO_X_L      0x3B
#define QMI8658_GYRO_X_H      0x3C
#define QMI8658_GYRO_Y_L      0x3D
#define QMI8658_GYRO_Y_H      0x3E
#define QMI8658_GYRO_Z_L      0x3F
#define QMI8658_GYRO_Z_H      0x40
#define QMI8658_TEMP_L        0x33
#define QMI8658_TEMP_H        0x34

static void QMI8658_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, QMI8658_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static uint8_t QMI8658_ReadReg(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c2, QMI8658_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

static void QMI8658_ReadMulti(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, QMI8658_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void QMI8658_Init(void)
{
    uint8_t id = QMI8658_ReadReg(QMI8658_WHO_AM_I);
    if (id != 0x05) {
        // 初始化失败，可在此处添加错误处理
        while(1);
    }

    // 软复位
    QMI8658_WriteReg(QMI8658_CTRL1, 0x80);
    HAL_Delay(10);

    // 配置加速度计：±8g，1000Hz输出
    QMI8658_WriteReg(QMI8658_CTRL2, 0x83);
    // 配置陀螺仪：±512dps，1000Hz输出
    QMI8658_WriteReg(QMI8658_CTRL3, 0x83);
    // 使能加速度计和陀螺仪
    QMI8658_WriteReg(QMI8658_CTRL1, 0x60);
}

void QMI8658_GetData(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[12];
    QMI8658_ReadMulti(QMI8658_ACCEL_X_L, data, 12);

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);
    *gx = (int16_t)((data[7] << 8) | data[6]);
    *gy = (int16_t)((data[9] << 8) | data[8]);
    *gz = (int16_t)((data[11] << 8) | data[10]);
}

int16_t QMI8658_GetTemperature(void)
{
    uint8_t data[2];
    QMI8658_ReadMulti(QMI8658_TEMP_L, data, 2);
    return (int16_t)((data[1] << 8) | data[0]);
}