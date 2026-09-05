#ifndef __ATTITUDE_H
#define __ATTITUDE_H

void PID_Attitude_Init(void) ; 
float PID_Attitude_Pitch(float target_angle, float measured_angle, float measured_gyro, float dt)  ; 
float PID_Attitude_Roll(float target_angle, float measured_angle, float measured_gyro, float dt)  ; 
float PID_Attitude_Yaw(float target_angle, float measured_angle, float measured_gyro, float dt)  ; 
void PID_Attitude_Reset(void) ; 
float PID_Rate_X(float measured, float target, float dt) ;
float PID_Rate_Y(float measured, float target, float dt) ;
float PID_Rate_Z(float measured, float target, float dt) ;
void pid_disable_ki(void) ;
void pid_enable_ki(void) ; 

#endif
