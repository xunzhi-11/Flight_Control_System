#include "app_task/app_task_parser.h"
#include "uart/ll_uart.h"
#include "crsf/CRSF_Parser.h"
#include "uart_ringbuf/uart1_ringbuf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

TaskHandle_t Parser_taskHandle = NULL ; 

static void Parser_Task(void * pvParameters)
{
    uint8_t rx_byte = 0 ; 
    
    while (1)
    { 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY) ;
        while (UART1_Update_Byte(&rx_byte)) 
        {
            CRSF_Parser_Byte(rx_byte) ;
        }
    }
    
}

void Task_Parser_Init(void)
{
    xTaskCreate(Parser_Task , "Parser_Task" , 256 , NULL , 5 , &Parser_taskHandle ) ; 
}

void UART1_IDEL_CallBack(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE ;

    // 确保任务已经创建
    if (Parser_taskHandle != NULL)
    {
        // 向指定的任务发送通知（让它的内部计数值 +1）
        vTaskNotifyGiveFromISR(Parser_taskHandle, &xHigherPriorityTaskWoken) ;
        
        // 触发上下文切换，保证极速响应
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken) ;
    }
}