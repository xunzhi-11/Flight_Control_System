#ifndef __PID_H
#define __PID_H

typedef struct {
    // 参数（可调）
    float kp ;
    float ki ;
    float kd ;
    float max_out ;      // 输出限幅
    float max_i_out ;    // 积分限幅（防积分饱和）

    // 运行时状态
    float prevError ;    // 上一次误差
    float prevMeasure ;  //上一次状态值
    float integral ;     // 积分累加值
    //微分滤波
    float lpf_beta;      // 低通滤波系数 (0~1)，越小滤波越强
    float prevDerivative; // 上一次的微分值
} PID_Controller ;

// 初始化函数
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float max_i_out) ;

// 计算函数，需要传入 dt (秒)
float PID_Compute_To_Error(PID_Controller *pid, float error, float dt) ;
float PID_Compute_To_Measure(PID_Controller *pid, float error, float measured, float dt) ;
void PID_Reset(PID_Controller* pid) ; 

#endif
