#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#include <stdint.h>

// 接收指令结构体（来自上位机）
typedef struct __attribute__((packed)) {
    float motor1_target_rps;
    float motor2_target_rps;
    uint16_t servo_angle;
} CommandPacket;

typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
    float motor1_actual_rps;
    float motor2_actual_rps;
} TelemetryPacket;

#endif // __COMM_PROTOCOL_H__