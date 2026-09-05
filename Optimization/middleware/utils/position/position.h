#ifndef __POSITION_H
#define __POSITION_H

#include "stdint.h"
#include "stdbool.h"
#include "data_structure_types.h"

void PID_PosHold_Init(void);
void PID_PosHold_Enable(const atgm336h_data_t *gps);
void PID_PosHold_Reset(void);
bool PID_PosHold_IsActive(void);
float PID_PosHold_StickNorm(uint16_t raw);
void PID_PosHold_Update(const atgm336h_data_t *gps,
                        float roll_stick_norm,
                        float pitch_stick_norm,
                        float yaw_deg,
                        float dt);
float PID_PosHold_GetRollAngle(void);
float PID_PosHold_GetPitchAngle(void);

#endif
