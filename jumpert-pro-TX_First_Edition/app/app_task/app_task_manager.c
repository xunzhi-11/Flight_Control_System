#include "app_task/app_task_manager.h"
#include "app_core/system_state.h"
#include "crsf/CRSF_Protocol.h"
#include "elrs/elrs_manager.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t ELRS_Manager_taskHandle = NULL ; 

void Manager_CallBack(System_Cmd_e cmd)
{
    if(ELRS_Manager_taskHandle != NULL)
    {
        xTaskNotify(ELRS_Manager_taskHandle, (uint32_t)cmd, eSetValueWithOverwrite) ;
    }
}
static void Manager_Task(void* pvParameters)
{
    STATE_Manager_RegisterCallback(Manager_CallBack) ; 
    uint32_t received_cmd = 0 ;
    while(1)
    {
        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &received_cmd, portMAX_DELAY) == pdTRUE)
        {
            switch ((System_Cmd_e)received_cmd)
            {
                case CMD_RF_UPDATE_PACKET_RATE:
                    {
                        ELRS_PacketRate_e new_rate = STATE_Get_RF_Param(ELRS_PARAM_PACKET_RATE) ;
                        ELRS_Set_Param(ELRS_PARAM_PACKET_RATE , new_rate) ; 
                    }
                break ;

                case CMD_RF_UPDATE_TX_POWER:
                    {
                        ELRS_MaxPower_e new_power = STATE_Get_RF_Param(ELRS_PARAM_MAX_POWER) ; 
                        ELRS_Set_Param(ELRS_PARAM_MAX_POWER , new_power) ; 
                    }
                break ;

                case CMD_RF_UPDATE_TELEM_RATIO:
                    {
                        ELRS_TELEM_RATIO_e new_ratio = STATE_Get_RF_Param(ELRS_PARAM_TELEM_RATIO) ;
                        ELRS_Set_Param(ELRS_PARAM_TELEM_RATIO, new_ratio) ;
                    }
                break ;

                case CMD_RF_START_BIND:
                    {
                        ELRS_Trigger_Command(ELRS_CMD_BIND) ; 
                    }
                break ;

                default:
                break ;
            }
        }

    }
}

void Task_Manager_Init(void)
{
    xTaskCreate(Manager_Task , "Manager_Task" , 256 , NULL , 5 , &ELRS_Manager_taskHandle) ; 
}

