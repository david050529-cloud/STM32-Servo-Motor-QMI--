#ifndef _QMI8658REG_H_
#define _QMI8658REG_H_

// 设备地址（SA0接地为0x6A，接高为0x6B）
#define QMI8658_ADDR                    0x6A

// 寄存器地址
enum Qmi8658Register {
    Qmi8658Register_WhoAmI          = 0x00,
    Qmi8658Register_Revision        = 0x01,
    Qmi8658Register_Ctrl1           = 0x02,
    Qmi8658Register_Ctrl2           = 0x03,
    Qmi8658Register_Ctrl3           = 0x04,
    Qmi8658Register_Ctrl4           = 0x05,
    Qmi8658Register_Ctrl5           = 0x06,
    Qmi8658Register_Ctrl6           = 0x07,
    Qmi8658Register_Ctrl7           = 0x08,
    Qmi8658Register_Ctrl8           = 0x09,
    Qmi8658Register_Ctrl9           = 0x0A,
    Qmi8658Register_Status0         = 0x2E,
    Qmi8658Register_Status1         = 0x2F,
    Qmi8658Register_Temp_L          = 0x33,
    Qmi8658Register_Temp_H          = 0x34,
    Qmi8658Register_Ax_L            = 0x35,
    Qmi8658Register_Ax_H            = 0x36,
    Qmi8658Register_Ay_L            = 0x37,
    Qmi8658Register_Ay_H            = 0x38,
    Qmi8658Register_Az_L            = 0x39,
    Qmi8658Register_Az_H            = 0x3A,
    Qmi8658Register_Gx_L            = 0x3B,
    Qmi8658Register_Gx_H            = 0x3C,
    Qmi8658Register_Gy_L            = 0x3D,
    Qmi8658Register_Gy_H            = 0x3E,
    Qmi8658Register_Gz_L            = 0x3F,
    Qmi8658Register_Gz_H            = 0x40,
    Qmi8658Register_Reset           = 0x60
};

// 加速度计量程
enum qmi8658_AccRange {
    Qmi8658AccRange_2g  = 0x00 << 4,
    Qmi8658AccRange_4g  = 0x01 << 4,
    Qmi8658AccRange_8g  = 0x02 << 4,
    Qmi8658AccRange_16g = 0x03 << 4
};

// 加速度计输出速率
enum qmi8658_AccOdr {
    Qmi8658AccOdr_8000Hz = 0x00,
    Qmi8658AccOdr_4000Hz = 0x01,
    Qmi8658AccOdr_2000Hz = 0x02,
    Qmi8658AccOdr_1000Hz = 0x03,
    Qmi8658AccOdr_500Hz  = 0x04,
    Qmi8658AccOdr_250Hz  = 0x05,
    Qmi8658AccOdr_125Hz  = 0x06,
    Qmi8658AccOdr_62_5Hz = 0x07,
    Qmi8658AccOdr_31_25Hz= 0x08
};

// 陀螺仪量程
enum qmi8658_GyrRange {
    Qmi8658GyrRange_16dps   = 0x00 << 4,
    Qmi8658GyrRange_32dps   = 0x01 << 4,
    Qmi8658GyrRange_64dps   = 0x02 << 4,
    Qmi8658GyrRange_128dps  = 0x03 << 4,
    Qmi8658GyrRange_256dps  = 0x04 << 4,
    Qmi8658GyrRange_512dps  = 0x05 << 4,
    Qmi8658GyrRange_1024dps = 0x06 << 4,
    Qmi8658GyrRange_2048dps = 0x07 << 4
};

// 陀螺仪输出速率
enum qmi8658_GyrOdr {
    Qmi8658GyrOdr_8000Hz = 0x00,
    Qmi8658GyrOdr_4000Hz = 0x01,
    Qmi8658GyrOdr_2000Hz = 0x02,
    Qmi8658GyrOdr_1000Hz = 0x03,
    Qmi8658GyrOdr_500Hz  = 0x04,
    Qmi8658GyrOdr_250Hz  = 0x05,
    Qmi8658GyrOdr_125Hz  = 0x06,
    Qmi8658GyrOdr_62_5Hz = 0x07,
    Qmi8658GyrOdr_31_25Hz= 0x08
};

// 低通滤波使能
enum qmi8658_LpfConfig {
    Qmi8658Lpf_Disable = 0,
    Qmi8658Lpf_Enable  = 1
};

// 自检使能
enum qmi8658_StConfig {
    Qmi8658St_Disable = 0,
    Qmi8658St_Enable  = 1
};

// 传感器使能掩码
#define QMI8658_ACC_ENABLE     0x01
#define QMI8658_GYR_ENABLE     0x02
#define QMI8658_ACCGYR_ENABLE  (QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)

#endif /* _QMI8658REG_H_ */