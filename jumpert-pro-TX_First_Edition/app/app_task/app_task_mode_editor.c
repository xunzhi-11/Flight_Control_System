#include "app_task/app_task_mode_editor.h"
#include "three_stage_lever/three_stage_lever.h"
#include "app_core/rc_data_center.h"
#include "FreeRTOS.h"
#include "task.h"

#define SW_TASK_MODE_EDITOR_INTERVAL 3U

TaskHandle_t mode_editor_taskHandle = NULL ;

static void task_mode_editor(void * pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_MODE_EDITOR_INTERVAL);

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        three_lever_mode_e flight_mode = get_three_lever_mode() ;

        flight_mode_update(flight_mode) ;
    }
}

void task_mode_editor_init(void)
{
    xTaskCreate(task_mode_editor , "task_mode_editor" , 256 , NULL , 8 , &mode_editor_taskHandle) ;
}
