#include "app_task/app_task_alt_hold.h"
#include "sys_software_config.h"
#include "crsf/CRSF_Parser.h"
#include "app_core/app_flight_mode.h"
#include "app_core/app_motor_unlocker.h"
#include "utils/altitude/altitude.h"
#include "FreeRTOS.h"
#include "task.h"

static crsf_channels_t s_channel;
static TaskHandle_t s_alt_hold_task_handle = NULL;

static void task_alt_hold(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_ALT_HOLD_INTERVAL);
    const float dt = SW_TASK_ALT_HOLD_INTERVAL / 1000.0f;

    (void)pvParameters;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!Safety_CanArm()) {
            continue;
        }

        {
            const FlightMode_e mode = FlightMode_Get();
            if (mode != FLIGHT_MODE_ALT_HOLD && mode != FLIGHT_MODE_STATIC) {
                continue;
            }
        }

        CRSF_ReadChannels_Control(&s_channel);
        PID_AltHold_Update(PID_AltHold_StickNorm(s_channel.ch2), dt);
    }
}

void task_alt_hold_init(void)
{
    PID_AltHold_Init();
    xTaskCreate(task_alt_hold,
                "task_alt_hold",
                256,
                NULL,
                SW_TASK_PRORITY_ALT_HOLD,
                &s_alt_hold_task_handle);
}
