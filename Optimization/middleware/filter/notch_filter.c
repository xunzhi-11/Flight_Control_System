#include "filter/notch_filter.h"
#include "data_structure_types.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 辅助函数：计算单组陷波滤波器系数
// f0:  陷波中心频率 (Hz)，若 <= 0 或 >= fs/2 则返回直通系数
// Q:   品质因数 (正数)
// fs:  采样频率 (Hz)
// 输出 b0,b1,b2,a1,a2 (对应差分方程: y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2)
static void notch_coeffs(float f0, float Q, float fs,
                         float *b0, float *b1, float *b2,
                         float *a1, float *a2)
{
    // 直通条件
    if (f0 <= 0.0f || Q <= 0.0f || f0 >= fs * 0.5f) {
        *b0 = 1.0f; *b1 = 0.0f; *b2 = 0.0f;
        *a1 = 0.0f; *a2 = 0.0f;
        return;
    }

    float omega0 = 2.0f * (float)M_PI * f0 / fs;      // 数字角频率
    float cos_omega0 = cosf(omega0);
    // 带宽 Δf = f0 / Q，对应数字域半径 r = exp(-π * f0 / (Q * fs))
    float r = expf(-(float)M_PI * f0 / (Q * fs));
    // 保证 r 小于 1 且稳定
    if (r >= 1.0f) r = 0.999f;

    // 分子系数 (1, -2cosω0, 1)
    *b0 = 1.0f;
    *b1 = -2.0f * cos_omega0;
    *b2 = 1.0f;
    // 分母系数 (1, -2r cosω0, r^2)
    *a1 = -2.0f * r * cos_omega0;
    *a2 = r * r;
}

void IMU_NotchFilter_Init(NotchFilter_t *lpf, float notch_freq_acc, float notch_freq_gyro, float Q, float fs)
{
    if (lpf == NULL) return;

    // 计算加速度计陷波系数
    notch_coeffs(notch_freq_acc, Q, fs,
                 &lpf->b0_acc, &lpf->b1_acc, &lpf->b2_acc,
                 &lpf->a1_acc, &lpf->a2_acc);
    // 计算陀螺仪陷波系数
    notch_coeffs(notch_freq_gyro, Q, fs,
                 &lpf->b0_gyro, &lpf->b1_gyro, &lpf->b2_gyro,
                 &lpf->a1_gyro, &lpf->a2_gyro);

    // 清零状态变量
    for (int i = 0; i < 6; i++) {
        lpf->x1[i] = 0.0f;
        lpf->x2[i] = 0.0f;
        lpf->y1[i] = 0.0f;
        lpf->y2[i] = 0.0f;
    }
    lpf->is_init = false;
}

void IMU_NotchFilter_Update(NotchFilter_t *lpf, const IMU_Scaled_t *scaled_data)
{
    if (lpf == NULL || scaled_data == NULL) return;

    // 原始输入转换为 float
    float in[6] = {
        (float)scaled_data->ax, (float)scaled_data->ay, (float)scaled_data->az,
        (float)scaled_data->gx, (float)scaled_data->gy, (float)scaled_data->gz
    };

    if (!lpf->is_init) {
        // 第一次运行：输出直接等于输入，并填充状态避免阶跃
        for (int i = 0; i < 6; i++) {
            lpf->out.ax = in[0]; lpf->out.ay = in[1]; lpf->out.az = in[2];
            lpf->out.gx = in[3]; lpf->out.gy = in[4]; lpf->out.gz = in[5];
            lpf->x1[i] = in[i];
            lpf->x2[i] = in[i];
            lpf->y1[i] = in[i];
            lpf->y2[i] = in[i];
        }
        lpf->is_init = true;
    } else {
        float out[6];

        // 加速度计通道 (0,1,2)
        for (int i = 0; i < 3; i++) {
            float b0 = lpf->b0_acc, b1 = lpf->b1_acc, b2 = lpf->b2_acc;
            float a1 = lpf->a1_acc, a2 = lpf->a2_acc;
            // 直接 I 型差分方程
            out[i] = b0 * in[i] + b1 * lpf->x1[i] + b2 * lpf->x2[i]
                     - a1 * lpf->y1[i] - a2 * lpf->y2[i];
        }

        // 陀螺仪通道 (3,4,5)
        for (int i = 3; i < 6; i++) {
            float b0 = lpf->b0_gyro, b1 = lpf->b1_gyro, b2 = lpf->b2_gyro;
            float a1 = lpf->a1_gyro, a2 = lpf->a2_gyro;
            out[i] = b0 * in[i] + b1 * lpf->x1[i] + b2 * lpf->x2[i]
                     - a1 * lpf->y1[i] - a2 * lpf->y2[i];
        }

        // 更新状态变量（保存当前输入/输出作为下一次的历史）
        for (int i = 0; i < 6; i++) {
            lpf->x2[i] = lpf->x1[i];
            lpf->x1[i] = in[i];
            lpf->y2[i] = lpf->y1[i];
            lpf->y1[i] = out[i];
        }

        // 写回输出结构体
        lpf->out.ax = out[0];
        lpf->out.ay = out[1];
        lpf->out.az = out[2];
        lpf->out.gx = out[3];
        lpf->out.gy = out[4];
        lpf->out.gz = out[5];
    }

    // 同步时间戳和采样间隔
    lpf->out.timestamp = scaled_data->timestamp;
    lpf->out.dt = scaled_data->dt;
}