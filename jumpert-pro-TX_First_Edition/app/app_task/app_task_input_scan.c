#include "app_task/app_task_input_scan.h"
#include "5_way_key/5_way_key.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t Input_Scan_taskHandle = NULL ; 

static QueueHandle_t s_action_queue = NULL ;

bool Input_Get_Action(Input_Action_e *out_action, uint32_t timeout_ms)
{
    if (s_action_queue == NULL) return false ;
    
    if (xQueueReceive(s_action_queue, out_action, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return true ;
    }
    return false ;
}

static void On_Key_Action_Triggered(Input_Action_e action)
{
    if (s_action_queue != NULL) 
    {
        xQueueSend(s_action_queue, &action, 0) ; 
    }
}

static void Input_Scan_Task(void* pvParameters)
{
    s_action_queue = xQueueCreate(10, sizeof(Input_Action_e)) ;

    Key_RegisterCallback(On_Key_Action_Triggered) ;

    TickType_t xLastWakeTime = xTaskGetTickCount() ;

    while(1)
    {
        Key_Scan() ;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)) ;
    }
}


void Task_InputScan_Init(void)
{
    xTaskCreate(Input_Scan_Task , "Input_Scan_Task" , 256 , NULL , 5 , &Input_Scan_taskHandle) ; 
}