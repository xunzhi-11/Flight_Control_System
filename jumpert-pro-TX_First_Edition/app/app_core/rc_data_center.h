#ifndef __RC_DATA_CENTER_H
#define __RC_DATA_CENTER_H

#include "stdint.h"
#include "crsf/CRSF_Telemetry.h"
#include "stdbool.h"
#include "system_hardware_config.h"
#include "three_stage_lever/three_stage_lever.h"

extern const uint8_t g_hw_adc_rank_to_rc_ch[HW_ADC_JOY_RANK_COUNT] ;

void joystick_channels_update(uint16_t* joystick_payload) ;
void RC_Data_Get_Snapshot(crsf_raw_channels_t *out_payload) ;
void motor_locker_update(bool ready_to_unlock) ;
void flight_mode_update(three_lever_mode_e flight_mode) ;


#endif
