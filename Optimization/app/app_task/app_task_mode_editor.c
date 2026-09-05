#include "app_task/app_task_mode_editor.h"
#include "sys_software_config.h"
#include "crsf/CRSF_Parser.h"
#include "app_core/app_motor_unlocker.h"
#include "app_core/app_flight_mode.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"

#define MODE_HOVER_VALUE     200U
#define MODE_MANUAL_VALUE    400U
#define MODE_POS_HOLD_VALUE  600U   /* was STATIC: outdoor GPS position hold */

static crsf_channels_t channel ;
TaskHandle_t mode_editor_taskHandle = NULL ;

static bool map_ch5_to_flight_mode(uint16_t raw, FlightMode_e *out_mode)
{
    switch (raw) {
    case MODE_HOVER_VALUE:
        *out_mode = FLIGHT_MODE_ALT_HOLD;
        return true;

    case MODE_MANUAL_VALUE:
        *out_mode = FLIGHT_MODE_MANUAL;
        return true;

    case MODE_POS_HOLD_VALUE:
        *out_mode = FLIGHT_MODE_STATIC;
        return true;

    default:
        return false;
    }
}

static void task_mode_editor(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_MODE_EDITOR_INTERVAL);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        CRSF_ReadChannels_Control(&channel);
        FlightMode_e mode ;

        if (map_ch5_to_flight_mode(channel.ch5, &mode)) {
            Safety_ClearBlock(ARM_BLOCK_MODE_FORBIDDEN);
            FlightMode_Set(mode);
        } else {
            Safety_SetBlock(ARM_BLOCK_MODE_FORBIDDEN);
        }
    }
}

void task_mode_editor_init(void)
{
    xTaskCreate(task_mode_editor, "task_mode_editor", 256, NULL, SW_TASK_PRORITY_MODE_EDITOR, &mode_editor_taskHandle);
}
