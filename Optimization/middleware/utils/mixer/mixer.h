#ifndef __MIXER_H
#define __MIXER_H

typedef struct
{
    float throttle; // 0 ~ 1
    float pitch;    // -1 ~ 1
    float roll;     // -1 ~ 1
    float yaw;      // -1 ~ 1
} MIXER_INPUT;

typedef struct
{
    float motor[4]; 
} MIXER_OUTPUT;

// 暴露推力补偿系数，方便在地面站或主逻辑中调节
extern float g_thrust_linearization_factor;

void Mixer_Update(const MIXER_INPUT* in, MIXER_OUTPUT* out);

#endif