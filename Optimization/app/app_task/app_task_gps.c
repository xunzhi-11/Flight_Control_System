#include "app_task/app_task_gps.h"
#include "sys_software_config.h"
#include "uart/ll_uart.h"
#include "nmea_0183/nmea_parser.h"
#include "FreeRTOS.h"
#include "task.h"

static void gps_task(void *pvParameters)
{
    uint8_t byte;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_GPS_INTERVAL);

    (void)pvParameters;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        while (UART3_Update_Byte(&byte))
        {
            NMEA_Parser_Byte(byte);
        }
    }
}

void task_gps_init(void)
{
    xTaskCreate(gps_task, "gps_task", 384, NULL, SW_TASK_PRIORITY_GPS, NULL);
}
