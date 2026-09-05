#include "app_task/app_task_motor_unlocker.h"
#include "two_stage_lever/two_statge_lever.h"
#include "app_core/rc_data_center.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"

#define SW_TASK_UNLOCKER_INTERVAL 3U

TaskHandle_t motor_unlocker_taskHandle = NULL ; 


static void task_motor_unlocker(void * pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_UNLOCKER_INTERVAL); 

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool redy_to_unlock = is_ready_to_unlock() ; 

        motor_locker_update(redy_to_unlock) ; 
    }
}

void task_motor_unlocker_init(void)
{
    xTaskCreate(task_motor_unlocker , "task_motor_unlocker" , 256 , NULL , 8 , &motor_unlocker_taskHandle) ; 
}