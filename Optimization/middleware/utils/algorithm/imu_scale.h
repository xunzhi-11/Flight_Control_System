#ifndef __IMU_SCALE_H
#define __IMU_SCALE_H

#include "stdint.h"

#define GYRO_SENSITIVITY_THEORETICAL         16.4f// 理论值: ±2000dps -> 16.4 LSB/dps
// 实测校正系数
// 原因可能是出厂灵敏度差异或内部滤波器衰减
#define GYRO_SCALE_CORRECTION                1.0f 
#define GYRO_SCALE_DPS                       (1.0f / GYRO_SENSITIVITY_THEORETICAL * GYRO_SCALE_CORRECTION)
#define DEG_TO_RAD                           (0.01745329251994329576923690768489f) // pi / 180
#define M_PI                                 (3.14159265358979323846f)
#define RAD_TO_DEG                           (57.295779513082320876798154814105f) // 180 / PI
#define ACCEL_SCALE                          (1.0f / 2048.0f) // LSB/g 的倒数，得到 g


static inline float acc_convert(int16_t acc_value)
{
    return acc_value * ACCEL_SCALE ; 
}

static inline float gyro_convert(int16_t gyro_value)
{
    return gyro_value * GYRO_SCALE_DPS ; 
}
#endif
