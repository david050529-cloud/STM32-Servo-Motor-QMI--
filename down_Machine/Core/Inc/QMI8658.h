#ifndef _QMI8658_H_
#define _QMI8658_H_

#include <stdint.h>
#include <stdbool.h>
#include "QMI8658reg.h"

// 欧拉角结构体（单位：度）
typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles_t;

// 初始化QMI8658，返回true表示成功
bool QMI8658_Init(void);

// 读取原始数据（加速度：g，角速度：dps）
// acc[3] = {ax, ay, az}, gyro[3] = {gx, gy, gz}
void QMI8658_ReadRaw(float acc[3], float gyro[3]);

// 读取温度（摄氏度）
float QMI8658_ReadTemperature(void);

// 获取欧拉角（需要周期性调用，内部使用互补滤波）
// 建议以固定周期（如10ms）调用，dt为两次调用间隔（秒）
void QMI8658_GetEuler(float dt, float *roll, float *pitch, float *yaw);

// 简便版：直接获取欧拉角（内部使用上次调用时间差，需在循环中周期调用）
void QMI8658_GetEulerSimple(float *roll, float *pitch, float *yaw);

#endif /* _QMI8658_H_ */