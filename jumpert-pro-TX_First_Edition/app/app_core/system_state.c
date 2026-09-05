#include "app_core/system_state.h"
#include "stddef.h"
#include "stdint.h"

#define CMD_TUNER_START  0x01
#define CMD_TUNER_END    0x20
#define CMD_SYSTEM_START 0x30
#define CMD_SYSTEM_END   0x50

typedef struct {
    ELRS_PacketRate_e packet_rate ; 
    uint16_t tx_power ;    
    ELRS_Bind_State_e  is_binding ;  // 0 或 1
} RF_Config_t ;

typedef struct {
    uint8_t backlight ;    // 0-100
    uint8_t batt_alarm ;   // 报警电压阈值
} Sys_Config_t ;

static uint8_t s_rf_params[20] = {0} ;
static Sys_Config_t s_sys_config = { .backlight = SYS_BACK_LIGHT_DEFAULT, .batt_alarm = SYS_BATT_ALARM_DEFAULT } ;

static state_callback_t s_state_manager_callback= NULL ;   
static state_callback_t s_state_system_callback= NULL ;   


void STATE_Manager_RegisterCallback(state_callback_t cb)
{
    s_state_manager_callback = cb ;
}

void STATE_System_RegisterCallback(state_callback_t cb)
{
    s_state_system_callback = cb ;
}

void STATE_Init(void)
{
    s_rf_params[ELRS_PARAM_PACKET_RATE] = PKT_RATE_50HZ; 
    s_rf_params[ELRS_PARAM_TELEM_RATIO]   = TELEM_RATIO_1_32; 
    s_rf_params[ELRS_PARAM_SWITCH_MODE] = SEND_MODE_8CH;
}

void SYS_Publish_Command(System_Cmd_e cmd)
{
    if(cmd >= CMD_TUNER_START && cmd <= CMD_TUNER_END)
    {
        if(s_state_manager_callback)
        {
            s_state_manager_callback(cmd) ; 
        }
    }
    else if (cmd >= CMD_SYSTEM_START && cmd <= CMD_SYSTEM_END)
    {
        if(s_state_system_callback)
        {
            s_state_system_callback(cmd) ; 
        }
    }
    
}

uint8_t STATE_Get_System_BackLight(void)
{
    return s_sys_config.backlight ; 
}

void STATE_Set_RF_Param(ELRS_Param_ID_e param_id, uint8_t value)
{
    if (param_id < 20) 
    {
        s_rf_params[param_id] = value ;
    }
}

uint8_t STATE_Get_RF_Param(ELRS_Param_ID_e param_id)
{
    if (param_id < 20) return s_rf_params[param_id] ;
    return 0 ;
}