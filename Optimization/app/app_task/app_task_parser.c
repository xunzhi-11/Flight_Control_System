#include "app_task/app_task_parser.h"   
#include "sys_software_config.h"
#include "uart/ll_uart.h"
#include "crsf/CRSF_Parser.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t parser_taskHandle = NULL ; 

static void parser_task(void* pvParameters)
{
    uint8_t byte;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_PARSER_INTERVAL); 

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        while (UART2_Update_Byte(&byte)) 
        {
            CRSF_Parser_Byte(byte); 
        }

    }
}

void task_parser_init(void)
{
    xTaskCreate(parser_task , "parser_task" , 256 , NULL , SW_TASK_PRIORITY_PARSER , &parser_taskHandle ) ; 
}


