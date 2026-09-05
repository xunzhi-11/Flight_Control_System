#include "app_core/ui_status_page.h"
#include "app_core/rc_data_center.h"
#include "crsf/CRSF_Parser.h"
#include "three_stage_lever/three_stage_lever.h"
#include "system_hardware_config.h"
#include "OLED/OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

#define MODULE_TIMEOUT_MS   2000U
#define TELEM_TIMEOUT_MS    1500U
#define CRSF_CH_MIN           172U
#define CRSF_CH_MAX          1811U

typedef struct {
    uint32_t last_seq ;
    TickType_t last_tick ;
} telem_track_t ;

static telem_track_t s_module_track = {0} ;
static telem_track_t s_link_track = {0} ;
static telem_track_t s_batt_track = {0} ;

static float crsf_throttle_norm(uint16_t raw)
{
    if (raw < CRSF_CH_MIN) raw = CRSF_CH_MIN ;
    if (raw > CRSF_CH_MAX) raw = CRSF_CH_MAX ;
    return (float)(raw - CRSF_CH_MIN) / (1811.0f - 172.0f) ;
}

static float crsf_climb_rate_norm(uint16_t raw)
{
    return crsf_throttle_norm(raw) * 2.0f - 1.0f ;
}

static void ui_status_format_signed_1dp(char *out, size_t cap, float val)
{
    int tenths = (int)(val * 10.0f + (val >= 0.0f ? 0.5f : -0.5f)) ;

    if (tenths < 0)
    {
        int abs_tenths = -tenths ;
        snprintf(out, cap, "-%d.%d", abs_tenths / 10, abs_tenths % 10) ;
    }
    else
    {
        snprintf(out, cap, "%d.%d", tenths / 10, tenths % 10) ;
    }
}

static void ui_status_format_mode_value(char *out, size_t cap, three_lever_mode_e mode, uint16_t throttle_raw)
{
    switch (mode)
    {
        case THREE_LEVER_MODE_HOVER :
        {
            char rate_buf[8] ;
            ui_status_format_signed_1dp(rate_buf, sizeof(rate_buf), crsf_climb_rate_norm(throttle_raw)) ;
            snprintf(out, cap, "CR:%s", rate_buf) ;
            break ;
        }

        case THREE_LEVER_MODE_MANUAL :
        {
            uint8_t pct = (uint8_t)(crsf_throttle_norm(throttle_raw) * 100.0f + 0.5f) ;
            snprintf(out, cap, "TH:%u%%", pct) ;
            break ;
        }

        case THREE_LEVER_MODE_STATIC :
        default :
            snprintf(out, cap, "static") ;
            break ;
    }
}

static bool ui_status_is_fresh(telem_track_t *track, uint32_t seq, uint32_t timeout_ms)
{
    TickType_t now = xTaskGetTickCount() ;

    if (seq != track->last_seq)
    {
        track->last_seq = seq ;
        track->last_tick = now ;
    }

    if (track->last_seq == 0)
    {
        return false ;
    }

    return (now - track->last_tick) <= pdMS_TO_TICKS(timeout_ms) ;
}

void UI_Status_Render(void)
{
    crsf_link_statistics_t link = *CRSF_Get_Link_Stats_Ptr() ;
    crsf_telemetry_battery_t batt = *CRSF_Get_Battery_Ptr() ;
    crsf_raw_channels_t rc = {0} ;
    three_lever_mode_e flight_mode = get_three_lever_mode() ;

    bool module_online = ui_status_is_fresh(&s_module_track, CRSF_Get_Module_Rx_Seq(), MODULE_TIMEOUT_MS) ;
    bool link_valid = ui_status_is_fresh(&s_link_track, CRSF_Get_Link_Stats_Seq(), TELEM_TIMEOUT_MS) ;
    bool batt_valid = ui_status_is_fresh(&s_batt_track, CRSF_Get_Battery_Seq(), TELEM_TIMEOUT_MS) ;
    bool rf_online = link_valid && link.uplink_LQ > 0 ;

    RC_Data_Get_Snapshot(&rc) ;
    uint16_t throttle_raw = rc.buffer[HW_RC_CH_THROTTLE] ;

    char line_buf[25] ;
    char mode_buf[12] ;
    ui_status_format_mode_value(mode_buf, sizeof(mode_buf), flight_mode, throttle_raw) ;

    OLED_Clear() ;
    OLED_ShowString(1, 1, "STATUS") ;

    snprintf(line_buf, sizeof(line_buf), "MOD:%s RF:%s",
             module_online ? "ON " : "OFF",
             rf_online ? "ON " : "OFF") ;
    OLED_ShowString(2, 1, line_buf) ;

    if (rf_online)
    {
        snprintf(line_buf, sizeof(line_buf), "LQ:%3u RSSI:-%u",
                 link.uplink_LQ,
                 link.uplink_RSSI_1) ;
    }
    else
    {
        snprintf(line_buf, sizeof(line_buf), "LQ:--- RSSI:---") ;
    }
    OLED_ShowString(3, 1, line_buf) ;

    if (batt_valid)
    {
        snprintf(line_buf, sizeof(line_buf), "%u.%uV %u%% %s",
                 batt.voltage_01v / 10U,
                 batt.voltage_01v % 10U,
                 batt.remaining_pct,
                 mode_buf) ;
    }
    else
    {
        snprintf(line_buf, sizeof(line_buf), "BAT:--- %s", mode_buf) ;
    }
    OLED_ShowString(4, 1, line_buf) ;

    OLED_ShowString(1, 10, "ENT>") ;
    OLED_Update() ;
}
