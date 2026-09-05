#ifndef __NOTCH_FILTER_H
#define __NOTCH_FILTER_H

#include "data_structure_types.h"
#include <stdbool.h>

typedef struct {
    // 加速度计系数 (b0, b1, b2, a1, a2)
    float b0_acc, b1_acc, b2_acc, a1_acc, a2_acc;
    // 陀螺仪系数
    float b0_gyro, b1_gyro, b2_gyro, a1_gyro, a2_gyro;

    // 状态变量: 每个通道存储 x[n-1], x[n-2], y[n-1], y[n-2]
    // 通道顺序: 0:ax, 1:ay, 2:az, 3:gx, 4:gy, 5:gz
    float x1[6], x2[6], y1[6], y2[6];

    bool is_init;
    IMU_Scaled_t out;
} NotchFilter_t;

// 初始化陷波滤波器
// lpf:           滤波器结构体指针
// notch_freq_acc: 加速度计陷波中心频率 (Hz)，若 <= 0 则直通
// notch_freq_gyro:陀螺仪陷波中心频率 (Hz)，若 <= 0 则直通
// Q:             品质因数 (Q = f0 / Δf)，Q 越大陷波越窄
// fs:            采样频率 (Hz) – 假设恒定，若实际采样率变化会引入误差
void IMU_NotchFilter_Init(NotchFilter_t *lpf, float notch_freq_acc, float notch_freq_gyro, float Q, float fs);

// 更新滤波器，输入原始 IMU 数据，输出滤波后的数据
void IMU_NotchFilter_Update(NotchFilter_t *lpf, const IMU_Scaled_t *scaled_data);

#endif // __NOTCH_FILTER_H