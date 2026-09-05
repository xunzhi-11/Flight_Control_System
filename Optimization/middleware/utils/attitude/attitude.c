#include "utils/attitude/attitude.h"
#include "utils/pid/pid.h"
#include "utils/madgwick/Madgwick_Fusion.h"


//内环参数
#define KP_RATE_PITCH   0.008f 
#define KI_RATE_PITCH   0.00000f  
#define KD_RATE_PITCH   0.00000f 


#define KP_RATE_ROLL   0.004f 
#define KI_RATE_ROLL   0.00000f  
#define KD_RATE_ROLL   0.00000f 

#define KP_RATE_YAW  0.0012f  
#define KI_RATE_YAW  0.00000f
#define KD_RATE_YAW  0.00000f 

#define MAX_OUT_RATE_PITCH  1.0f
#define MAX_OUT_I_RATE_PITCH  0.5f
#define MAX_OUT_RATE_ROLL  0.5f
#define MAX_OUT_I_RATE_ROLL  0.5f
#define MAX_OUT_RATE_YAW  0.25f
#define MAX_OUT_I_RATE_YAW  0.25f


//外环参数
#define KP_ANGLE_PITCH  1.6f
#define KI_ANGLE_PITCH  0.05f
#define KD_ANGLE_PITCH  0.0f

#define KP_ANGLE_ROLL  1.6f
#define KI_ANGLE_ROLL  0.05f
#define KD_ANGLE_ROLL  0.0f

#define KP_ANGLE_YAW  1.20f
#define KI_ANGLE_YAW  0.000f
#define KD_ANGLE_YAW  0.00f

#define MAX_OUT_ANGLE_PITCH  120.0f
#define MAX_OUT_I_ANGLE_PITCH  10.0f
#define MAX_OUT_ANGLE_ROLL  120.0f
#define MAX_OUT_I_ANGLE_ROLL  10.0f
#define MAX_OUT_ANGLE_YAW 60.0f
#define MAX_OUT_I_ANGLE_YAW  5.0f


//外环角度环
PID_Controller pid_pitch_angle ; 
PID_Controller pid_roll_angle  ; 
PID_Controller pid_yaw_angle ; 
//内环角速度环
PID_Controller pid_pitch_rate ; 
PID_Controller pid_roll_rate  ; 
PID_Controller pid_yaw_rate ; 

void PID_Attitude_Init(void)
{    
    PID_Init(&pid_pitch_angle, KP_ANGLE_PITCH , KI_ANGLE_PITCH , KD_ANGLE_PITCH, MAX_OUT_ANGLE_PITCH, MAX_OUT_I_ANGLE_PITCH); 
    PID_Init(&pid_roll_angle, KP_ANGLE_ROLL , KI_ANGLE_ROLL , KD_ANGLE_ROLL, MAX_OUT_ANGLE_ROLL, MAX_OUT_I_ANGLE_ROLL); 
    PID_Init(&pid_yaw_angle, KP_ANGLE_YAW , KI_ANGLE_YAW , KD_ANGLE_YAW, MAX_OUT_ANGLE_YAW, MAX_OUT_I_ANGLE_YAW);

    PID_Init(&pid_pitch_rate, KP_RATE_PITCH , KI_RATE_PITCH, KD_RATE_PITCH, MAX_OUT_RATE_PITCH, MAX_OUT_I_RATE_PITCH); 
    PID_Init(&pid_roll_rate, KP_RATE_ROLL , KI_RATE_ROLL, KD_RATE_ROLL, MAX_OUT_RATE_ROLL, MAX_OUT_I_RATE_ROLL); 
    PID_Init(&pid_yaw_rate, KP_RATE_YAW , KI_RATE_YAW, KD_RATE_YAW, MAX_OUT_RATE_YAW, MAX_OUT_I_RATE_YAW);
}

void pid_disable_ki(void)
{
    pid_pitch_rate.ki = 0.0f;
    pid_roll_rate.ki = 0.0f;
    pid_yaw_rate.ki = 0.0f;
}

void pid_enable_ki(void)
{
    pid_pitch_rate.ki = KI_RATE_PITCH;
    pid_roll_rate.ki = KI_RATE_ROLL;
    pid_yaw_rate.ki = KI_RATE_YAW;
}


static float Calculate_Angle_Error(float target, float measured) {
    float error = target - measured;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

float PID_Attitude_Pitch(float target_angle, float measured_angle, float measured_gyro, float dt)
{
    float error_angle = target_angle - measured_angle;
    float target_rate = PID_Compute_To_Error(&pid_pitch_angle, error_angle, dt);
    float error_rate = target_rate - measured_gyro;
    return PID_Compute_To_Measure(&pid_pitch_rate, error_rate, measured_gyro, dt);
}

float PID_Attitude_Roll(float target_angle, float measured_angle, float measured_gyro, float dt)
{
    float error_angle = target_angle - measured_angle; 

    float target_rate = PID_Compute_To_Error(&pid_roll_angle, error_angle, dt);

    float error_rate = target_rate - measured_gyro;

    return PID_Compute_To_Measure(&pid_roll_rate, error_rate, measured_gyro, dt);
}

float PID_Attitude_Yaw(float target_angle, float measured_angle, float measured_gyro, float dt)
{
    // Yaw 用 Calculate_Angle_Error
    float error_angle = Calculate_Angle_Error(target_angle , measured_angle) ; 
    float target_rate = PID_Compute_To_Error(&pid_yaw_angle, error_angle, dt);

    float error_rate = target_rate - measured_gyro;

    return PID_Compute_To_Measure(&pid_yaw_rate, error_rate, measured_gyro, dt);
}

float PID_Rate_X(float measured, float target, float dt) 
{
    float error = target - measured; 
    return PID_Compute_To_Measure(&pid_roll_rate, error, measured, dt); 
}
float PID_Rate_Y(float measured, float target, float dt) 
{
    float error = target - measured; 
    return PID_Compute_To_Measure(&pid_pitch_rate, error, measured, dt); 
}

float PID_Rate_Z(float measured, float target, float dt) 
{
    float error = target - measured; 
    return PID_Compute_To_Measure(&pid_yaw_rate, error, measured, dt); 
}

void PID_Attitude_Reset(void)
{
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_roll_angle);
    PID_Reset(&pid_yaw_angle);
    
    PID_Reset(&pid_pitch_rate);
    PID_Reset(&pid_roll_rate);
    PID_Reset(&pid_yaw_rate);
}