#ifndef __SYSTEM_STATE_H
#define __SYSTEM_STATE_H

#include "app_core/system_config_types.h"

typedef enum {
    CMD_NONE = 0x00,
    
    // 高频头相关命令
    CMD_RF_UPDATE_PACKET_RATE = 0x01 ,
    CMD_RF_UPDATE_TX_POWER = 0x02 ,
    CMD_RF_START_BIND = 0x03 ,
    CMD_RF_UPDATE_TELEM_RATIO = 0x04 ,
    
    // 系统相关命令
    CMD_SYS_START_JOY_CALIB = 0x30,
    CMD_SYS_SAVE_CONFIG = 0x31
} System_Cmd_e ;

typedef void (*state_callback_t)(System_Cmd_e cmd) ;


void SYS_Publish_Command(System_Cmd_e cmd) ;
void STATE_Set_RF_Param(ELRS_Param_ID_e param_id, uint8_t value) ; 
uint8_t STATE_Get_RF_Param(ELRS_Param_ID_e param_id) ; 
void STATE_Init(void) ; 
uint8_t STATE_Get_System_BackLight(void) ; 
void STATE_Manager_RegisterCallback(state_callback_t cb) ; 
void STATE_System_RegisterCallback(state_callback_t cb) ; 

#endif
