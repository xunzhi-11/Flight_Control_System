#include "app_core/rc_data_center.h"
#include "three_stage_lever/three_stage_lever.h"
#include "system_hardware_config.h"
#include "FreeRTOS.h"
#include "stdbool.h"
#include "task.h"
#include "string.h"

#define CH4_LOCK_VALUE             0U
#define CH4_UNLOCK_VALUE           1500U
#define CH5_HOVER_VALUE            200U
#define CH5_STATIC_VALUE           600U
#define CH5_MANUAL_VALUE           400U

const uint8_t g_hw_adc_rank_to_rc_ch[HW_ADC_JOY_RANK_COUNT] = HW_ADC_RANK_TO_RC_CH_TABLE ;

crsf_raw_channels_t rc_channels_data = {0} ; 

void RC_Data_Get_Snapshot(crsf_raw_channels_t *out_payload)
{
    taskENTER_CRITICAL() ; 
    *out_payload = rc_channels_data ; 
    taskEXIT_CRITICAL() ;  
}

void joystick_channels_update(uint16_t* joystick_payload)
{
    taskENTER_CRITICAL() ; 
    rc_channels_data.buffer[0] = joystick_payload[0] ; 
    rc_channels_data.buffer[1] = joystick_payload[1] ; 
    rc_channels_data.buffer[2] = joystick_payload[2] ; 
    rc_channels_data.buffer[3] = joystick_payload[3] ; 
    taskEXIT_CRITICAL() ;  
}

void motor_locker_update(bool ready_to_unlock)
{
    switch (ready_to_unlock)
    {
        case true :
        rc_channels_data.buffer[4] = CH4_LOCK_VALUE ; 
        break ;

        case false :
        rc_channels_data.buffer[4] = CH4_UNLOCK_VALUE ; 
        break ;
    }
}

void flight_mode_update(three_lever_mode_e flight_mode)
{
    switch (flight_mode)
    {
        case THREE_LEVER_MODE_HOVER :
        rc_channels_data.buffer[5] = CH5_HOVER_VALUE ;
        break ;

        case THREE_LEVER_MODE_STATIC :
        rc_channels_data.buffer[5] = CH5_STATIC_VALUE ;
        break ;

        case THREE_LEVER_MODE_MANUAL :
        rc_channels_data.buffer[5] = CH5_MANUAL_VALUE ;
        break ;

        default :
        rc_channels_data.buffer[5] = CH5_MANUAL_VALUE ;
        break ;
    }
}