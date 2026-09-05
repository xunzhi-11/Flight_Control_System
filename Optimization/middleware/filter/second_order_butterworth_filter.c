#include "filter/second_order_butterworth_filter.h"
#include "data_structure_types.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 辅助函数：计算单组二阶巴特沃斯系数 (双线性变换法)
// fc:  截止频率 (Hz)
// fs:  采样频率 (Hz)
// 输出 b0, b1, b2, a1, a2 (对应差分方程: y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2)
static void butterworth_coeffs(float fc, float fs,
                               float *b0, float *b1, float *b2,
                               float *a1, float *a2)
{
    if (fc <= 0.0f || fs <= 0.0f) {
        // 无效频率：系数置为直通滤波器 (y[n] = x[n])
        *b0 = 1.0f; *b1 = 0.0f; *b2 = 0.0f;
        *a1 = 0.0f; *a2 = 0.0f;
        return;
    }

    // 限制截止频率 <= 奈奎斯特频率
    if (fc >= fs * 0.5f) fc = fs * 0.5f - 1e-6f;

    float K = tanf((float)M_PI * fc / fs);
    float K2 = K * K;
    float sqrt2 = 1.4142135623730951f; // sqrt(2)

    float denom = 1.0f + sqrt2 * K + K2;
    *b0 = K2 / denom;
    *b1 = 2.0f * K2 / denom;
    *b2 = K2 / denom;
    *a1 = 2.0f * (K2 - 1.0f) / denom;
    *a2 = (1.0f - sqrt2 * K + K2) / denom;
}

void IMU_ButterworthLPF_Init(ButterworthLPF_t *lpf, float fc_acc, float fc_gyro, float fs)
{
    if (lpf == NULL) return;

    // 计算加速度计系数
    butterworth_coeffs(fc_acc, fs,
                       &lpf->b0_acc, &lpf->b1_acc, &lpf->b2_acc,
                       &lpf->a1_acc, &lpf->a2_acc);
    // 计算陀螺仪系数
    butterworth_coeffs(fc_gyro, fs,
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

void IMU_ButterworthLPF_Update(ButterworthLPF_t *lpf, const IMU_Scaled_t *scaled_data)
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
            // 所有历史状态设为当前输入值，使滤波器初始输出平稳
            lpf->x1[i] = in[i];
            lpf->x2[i] = in[i];
            lpf->y1[i] = in[i];
            lpf->y2[i] = in[i];
        }
        lpf->is_init = true;
    } else {
        float out[6];
        // 分别处理加速度计 (通道0~2) 和陀螺仪 (通道3~5)
        for (int i = 0; i < 3; i++) { // 加速度计通道
            float b0 = lpf->b0_acc, b1 = lpf->b1_acc, b2 = lpf->b2_acc;
            float a1 = lpf->a1_acc, a2 = lpf->a2_acc;
            // 直接 I 型差分方程: y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
            out[i] = b0 * in[i] + b1 * lpf->x1[i] + b2 * lpf->x2[i]
                     - a1 * lpf->y1[i] - a2 * lpf->y2[i];
        }
        for (int i = 3; i < 6; i++) { // 陀螺仪通道
            float b0 = lpf->b0_gyro, b1 = lpf->b1_gyro, b2 = lpf->b2_gyro;
            float a1 = lpf->a1_gyro, a2 = lpf->a2_gyro;
            out[i] = b0 * in[i] + b1 * lpf->x1[i] + b2 * lpf->x2[i]
                     - a1 * lpf->y1[i] - a2 * lpf->y2[i];
        }

        // 更新状态变量 (保存当前输入和输出作为下一次的历史)
        for (int i = 0; i < 6; i++) {
            lpf->x2[i] = lpf->x1[i];
            lpf->x1[i] = in[i];
            lpf->y2[i] = lpf->y1[i];
            lpf->y1[i] = out[i];
        }

        // 将结果写回输出结构体
        lpf->out.ax = out[0];
        lpf->out.ay = out[1];
        lpf->out.az = out[2];
        lpf->out.gx = out[3];
        lpf->out.gy = out[4];
        lpf->out.gz = out[5];
    }

    lpf->out.timestamp = scaled_data->timestamp;
    lpf->out.dt = scaled_data->dt;
}