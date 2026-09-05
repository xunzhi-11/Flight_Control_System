#include "app_task/app_task_telemetry.h"
#include "tim/ll_tim3.h"
#include "app_core/rc_data_center.h"
#include "crsf/CRSF_Telemetry.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t Telemetry_taskHandle = NULL ; 

void TIM3_CallBack(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE ;

    if(Telemetry_taskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(Telemetry_taskHandle, &xHigherPriorityTaskWoken) ;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken) ;
    }
}


static void Tele_Task(void * pvParameters)
{
    TIM3_RegisterCallback(TIM3_CallBack) ; 

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY) ;
        crsf_raw_channels_t payload ; 
        RC_Data_Get_Snapshot(&payload) ; 
        CRSF_Send_RC_Channel(&payload) ; 
    }

}

void Task_Tele_Init(void)
{
    xTaskCreate(Tele_Task , "Tele_Task" , 256 , NULL , 5 , &Telemetry_taskHandle) ; 
}

