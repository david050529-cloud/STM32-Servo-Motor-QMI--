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
    uint8_t  header[2];          // 帧头: 0xAA, 0x55
    float roll;
    float pitch;
    float yaw;
    float motor1_actual_rps;
    float motor2_actual_rps;
    uint8_t  footer[2];          // 帧尾: 0x0D, 0x0A
} TelemetryPacket;

#endif // __COMM_PROTOCOL_H__