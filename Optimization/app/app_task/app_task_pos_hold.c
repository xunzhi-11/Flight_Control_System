#include "app_task/app_task_pos_hold.h"
#include "sys_software_config.h"
#include "crsf/CRSF_Parser.h"
#include "app_core/app_flight_mode.h"
#include "app_core/app_motor_unlocker.h"
#include "app_core/app_system_data_center.h"
#include "utils/madgwick/Madgwick_Fusion.h"
#include "utils/position/position.h"
#include "FreeRTOS.h"
#include "task.h"

static crsf_channels_t s_channel;
static atgm336h_data_t s_gps;
static TaskHandle_t s_pos_hold_task_handle = NULL;

static void task_pos_hold(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SW_TASK_POS_HOLD_INTERVAL);
    const float dt = SW_TASK_POS_HOLD_INTERVAL / 1000.0f;

    (void)pvParameters;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!Safety_CanArm()) {
            continue;
        }

        if (FlightMode_Get() != FLIGHT_MODE_STATIC) {
            continue;
        }

        gps_get_snapshot(&s_gps);
        CRSF_ReadChannels_Control(&s_channel);

        PID_PosHold_Update(&s_gps,
                           PID_PosHold_StickNorm(s_channel.ch0),
                           PID_PosHold_StickNorm(s_channel.ch1),
                           MadgwickFusion_GetStatePtr()->euler.yaw,
                           dt);
    }
}

void task_pos_hold_init(void)
{
    PID_PosHold_Init();
    xTaskCreate(task_pos_hold,
                "task_pos_hold",
                256,
                NULL,
                SW_TASK_PRORITY_POS_HOLD,
                &s_pos_hold_task_handle);
}
