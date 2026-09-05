#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#include "stdint.h"

void PID_AltHold_Init(void);
void PID_AltHold_Enable(float current_manual_throttle);
void PID_AltHold_Reset(void);
float PID_AltHold_StickNorm(uint16_t raw);
float PID_AltHold_Update(float rc_stick_norm, float dt);
float PID_AltHold_GetThrottle(void);

#endif
