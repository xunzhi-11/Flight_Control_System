#include "app_task/app_task_motor_unlocker.h"
#include "sys_software_config.h"
#include "crsf/CRSF_Parser.h"
#include "app_core/app_motor_unlocker.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"


static crsf_channels_t channel ; 
TaskHandle_t motor_unlocker_taskHandle = NULL ; 



static void task_motor_unlocker(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_UNLOCKER_INTERVAL); 

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        CRSF_ReadChannels_Control(&channel) ;
        uint16_t redy_to_unlock = channel.ch4 ; 

        switch(redy_to_unlock > MOTOR_UNLOCKER_BOUNDARY_VALUE)
        {
            case true :
            Safety_SetBlock(ARM_BLOCK_SWITCH_OFF) ; 
            break ;

            case false :
            Safety_ClearBlock(ARM_BLOCK_SWITCH_OFF) ; 
            break ;
        }


    }
}

void task_motor_unlocker_init(void)
{
    xTaskCreate(task_motor_unlocker , "task_motor_unlocker" , 256 , NULL , SW_TASK_PRORITY_MOTOR_UNLOCKER , &motor_unlocker_taskHandle) ; 
}