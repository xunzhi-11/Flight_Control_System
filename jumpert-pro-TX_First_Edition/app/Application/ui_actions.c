#include "Application/ui_actions.h"
#include "app_core/system_state.h"
#include "app_core/joy_calib_flow.h"
#include "stdint.h"

void Action_Set_PacketRate(int32_t rate) 
{
    STATE_Set_RF_Param( ELRS_PARAM_PACKET_RATE, (ELRS_PacketRate_e)rate) ;
    SYS_Publish_Command(CMD_RF_UPDATE_PACKET_RATE) ;
    UI_Show_Toast("Rate Saved") ; 
}

void Action_ELRS_Bind(int32_t arg)
{
    SYS_Publish_Command(CMD_RF_START_BIND) ;
    UI_Show_Toast("Start Bind") ; 
}

void Action_Start_Joy_Calib(int32_t arg)
{
    (void)arg ;
    JoyCalib_Start() ;
}

void Action_Set_TX_Power(int32_t power)
{
    STATE_Set_RF_Param(ELRS_PARAM_MAX_POWER , power) ; 
    SYS_Publish_Command(CMD_RF_UPDATE_TX_POWER) ; 
    UI_Show_Toast("Power Saved") ; 
}

void Action_Set_TelemRatio(int32_t ratio)
{
    STATE_Set_RF_Param(ELRS_PARAM_TELEM_RATIO, (uint8_t)ratio) ;
    SYS_Publish_Command(CMD_RF_UPDATE_TELEM_RATIO) ;
    UI_Show_Toast("Telem Saved") ;
}
