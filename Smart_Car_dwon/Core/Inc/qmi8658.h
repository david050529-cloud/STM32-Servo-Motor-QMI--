/* qmi8658.h */
#ifndef QMI8658_H
#define QMI8658_H

#include "main.h"

#define QMI8658_ADDR  0x6B  // I2C地址（AD0接地）

void QMI8658_Init(void);
void QMI8658_GetData(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
int16_t QMI8658_GetTemperature(void);

#endif