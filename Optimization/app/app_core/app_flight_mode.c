#include "app_core/app_flight_mode.h"
#include "app_core/app_system_data_center.h"
#include "utils/altitude/altitude.h"
#include "utils/position/position.h"
#include "crsf/CRSF_Parser.h"

static volatile FlightMode_e s_flight_mode = FLIGHT_MODE_MANUAL;

static void flight_mode_on_exit(FlightMode_e mode)
{
    if (mode == FLIGHT_MODE_ALT_HOLD || mode == FLIGHT_MODE_STATIC)
    {
        PID_AltHold_Reset();
    }

    if (mode == FLIGHT_MODE_STATIC)
    {
        PID_PosHold_Reset();
    }
}

static float flight_mode_throttle_norm(uint16_t raw)
{
    if (raw < 172) raw = 172;
    if (raw > 1811) raw = 1811;
    return (float)(raw - 172) / (1811.0f - 172.0f);
}

static void flight_mode_on_enter(FlightMode_e mode)
{
    if (mode == FLIGHT_MODE_ALT_HOLD || mode == FLIGHT_MODE_STATIC)
    {
        crsf_channels_t ch;
        CRSF_ReadChannels_Control(&ch);
        PID_AltHold_Enable(flight_mode_throttle_norm(ch.ch2));
    }

    if (mode == FLIGHT_MODE_STATIC)
    {
        atgm336h_data_t gps;
        gps_get_snapshot(&gps);
        PID_PosHold_Enable(&gps);
    }
}

void FlightMode_Init(void)
{
    s_flight_mode = FLIGHT_MODE_MANUAL;
}

FlightMode_e FlightMode_Get(void)
{
    return s_flight_mode;
}

void FlightMode_Set(FlightMode_e mode)
{
    if (mode >= FLIGHT_MODE_COUNT) {
        return;
    }

    if (mode == s_flight_mode) {
        return;
    }

    FlightMode_e prev = s_flight_mode;
    s_flight_mode = mode;
    flight_mode_on_exit(prev);
    flight_mode_on_enter(mode);
}
