#include "utils/pid/pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float max_i_out) 
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_i_out = max_i_out;
    pid->prevError = 0.0f;
    pid->prevMeasure = 0.0f;
    pid->integral = 0.0f;
    pid->prevDerivative = 0.0f ; 
    pid->lpf_beta = 0.25f ; 
}

// -----------------------------------------------------------
// 标准 PID：微分项基于误差变化 (D = Kd * d(Error)/dt)
// 适用：外环（角度环），因为设定值通常变化平缓
// -----------------------------------------------------------
float PID_Compute_To_Error(PID_Controller *pid, float error, float dt) 
{
    // 这里直接传入 error，而不是 target 和 measured
    // 这样角度归一化可以在外部处理，更加灵活

    // P
    float P = pid->kp * error;

   // I 
    pid->integral += pid->ki * error * dt;

    if (pid->integral > pid->max_i_out) 
    {
        pid->integral = pid->max_i_out;
    }
    else if (pid->integral < -pid->max_i_out) 
    {
        pid->integral = -pid->max_i_out;
    }
    
    float I = pid->integral;
    // D (基于误差)
    float derivative = (error - pid->prevError) / dt;
    float D = pid->kd * derivative;
    
    pid->prevError = error;

    float output = P + I + D;

    // 输出限幅
    if (output > pid->max_out) output = pid->max_out;
    else if (output < -pid->max_out) output = -pid->max_out;

    return output;
}

// -----------------------------------------------------------
// 微分先行 PID：微分项基于测量值变化 (D = -Kd * d(Measure)/dt)
// 适用：内环（角速度环），因为设定值(遥控器)会阶跃变化，
// 基于测量值的微分可以避免设定值突变带来的 D 项冲击
// -----------------------------------------------------------
float PID_Compute_To_Measure(PID_Controller *pid, float error, float measured, float dt) 
{
    // P
    float P = pid->kp * error;
    // I
    pid->integral += pid->ki * error * dt;

    // 对最终的积分输出直接进行限幅
    if (pid->integral > pid->max_i_out) 
    {
        pid->integral = pid->max_i_out;
    }
    else if (pid->integral < -pid->max_i_out) 
    {
        pid->integral = -pid->max_i_out;
    }
    
    // I 项的结果就是累加器本身
    float I = pid->integral;
    // D 基于测量值
    
    // 计算原始微分 (Raw Derivative)
    float raw_derivative = (measured - pid->prevMeasure) / dt;

    // 低通滤波 (Low Pass Filter)
    // 公式: Out = Prev + Beta * (Raw - Prev)
    // 如果 beta 是 1.0，表示不滤波；如果 beta 是 0.1，表示强滤波
    float derivative = pid->prevDerivative + pid->lpf_beta * (raw_derivative - pid->prevDerivative);
    
    // 更新状态
    pid->prevDerivative = derivative;
    pid->prevMeasure = measured;

    // 计算最终 D 项
    float D = pid->kd * derivative;

    float output = P + I - D; 

    // 输出限幅
    if (output > pid->max_out) output = pid->max_out;
    else if (output < -pid->max_out) output = -pid->max_out;

    return output;
}

void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f ; 
    pid->prevDerivative = 0.0f ; 
    pid->prevError = 0.0f ; 
    pid->prevMeasure = 0.0f ; 
}

