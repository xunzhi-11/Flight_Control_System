#include "utils/mixer/mixer.h"
#include <math.h> // for fmax, sqrtf

#define MIXER_MAX_OUT  1.0f 
#define MIXER_MIN_OUT 0.0f

// 默认补偿系数，假设为0.5。根据实际电机/桨叶的拉力测试数据进行修改
float g_thrust_linearization_factor = 0.3f; 

/**
 * 逆推力模型：将期望的线性推力 (0~1) 转换为非线性的电机控制指令 (0~1)
 */
static float ApplyThrustLinearization(float target_thrust) 
{
    float a = g_thrust_linearization_factor;
    
    // 如果系数极小，等同于没有非线性，直接返回
    if (a < 0.01f) {
        return target_thrust;
    }
    
    // 安全限制，防止后续数学计算溢出或出现虚数
    if (target_thrust <= 0.0f) return 0.0f;
    if (target_thrust >= 1.0f) return 1.0f;

    // 根据二次拟合方程: T = a*M^2 + (1-a)*M 求解 M
    // 使用求根公式: M = ( -(1-a) + sqrt((1-a)^2 + 4*a*T) ) / (2*a)
    float one_minus_a = 1.0f - a;
    float M = (-one_minus_a + sqrtf(one_minus_a * one_minus_a + 4.0f * a * target_thrust)) / (2.0f * a);
    
    return M;
}

void Mixer_Update(const MIXER_INPUT* in, MIXER_OUTPUT* out) {
    float m[4];
    m[0] = in->throttle + in->pitch + in->roll + in->yaw;
    m[1] = in->throttle + in->pitch - in->roll - in->yaw;
    m[2] = in->throttle - in->pitch - in->roll + in->yaw;
    m[3] = in->throttle - in->pitch + in->roll - in->yaw;
    
    float max_val = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > max_val) max_val = m[i];
    }
    
    if (max_val > 1.0f) {
        float excess = max_val - 1.0f;
        float attitude_scale = 1.0f - (excess / max_val);
        
        for (int i = 0; i < 4; i++) {
            out->motor[i] = in->throttle + (m[i] - in->throttle) * attitude_scale;
        }
    } else {
        for (int i = 0; i < 4; i++) {
            out->motor[i] = m[i];
        }
    }
    
    // 上下限处理
    for (int i = 0; i < 4; i++) {
        if (out->motor[i] < MIXER_MIN_OUT) out->motor[i] = MIXER_MIN_OUT;
        if (out->motor[i] > MIXER_MAX_OUT)  out->motor[i] = MIXER_MAX_OUT;
    }
    
    // --- 推力线性化映射 ---
    // 将线性的"期望推力"转换为非线性的"电机控制指令"
    for (int i = 0; i < 4; i++) {
        out->motor[i] = ApplyThrustLinearization(out->motor[i]);
    }
}