#include "app_core/app_system_data_center.h"

static atgm336h_data_t s_gps_work;
static atgm336h_data_t s_gps_pub[2];
static volatile uint8_t s_gps_pub_active;

static void gps_publish(void)
{
    uint8_t inactive = s_gps_pub_active ^ 1u;
    s_gps_pub[inactive] = s_gps_work;
    s_gps_pub_active = inactive;
}

void gps_init(void)
{
    s_gps_work = (atgm336h_data_t){0};
    s_gps_pub[0] = (atgm336h_data_t){0};
    s_gps_pub[1] = (atgm336h_data_t){0};
    s_gps_pub_active = 0;
}

void gps_get_snapshot(atgm336h_data_t *out)
{
    if (out == NULL)
    {
        return;
    }

    uint8_t active = s_gps_pub_active;
    *out = s_gps_pub[active];
}

void gps_apply_gga(const atgm336h_data_t *partial)
{
    if (partial == NULL)
    {
        return;
    }

    s_gps_work.hour = partial->hour;
    s_gps_work.minute = partial->minute;
    s_gps_work.second = partial->second;
    s_gps_work.msec = partial->msec;
    s_gps_work.latitude = partial->latitude;
    s_gps_work.longitude = partial->longitude;
    s_gps_work.altitude = partial->altitude;
    s_gps_work.fix_quality = partial->fix_quality;
    s_gps_work.satellites = partial->satellites;
    s_gps_work.hdop = partial->hdop;

    if (partial->fix_quality >= 1U)
    {
        s_gps_work.is_valid = true;
    }
    else
    {
        s_gps_work.is_valid = false;
    }

    gps_publish();
}

void gps_apply_rmc(const atgm336h_data_t *partial)
{
    if (partial == NULL)
    {
        return;
    }

    s_gps_work.hour = partial->hour;
    s_gps_work.minute = partial->minute;
    s_gps_work.second = partial->second;
    s_gps_work.msec = partial->msec;
    s_gps_work.year = partial->year;
    s_gps_work.month = partial->month;
    s_gps_work.day = partial->day;
    s_gps_work.latitude = partial->latitude;
    s_gps_work.longitude = partial->longitude;
    s_gps_work.speed_kmh = partial->speed_kmh;
    s_gps_work.course = partial->course;
    s_gps_work.is_valid = partial->is_valid;

    gps_publish();
}

void gps_to_crsf_telemetry(const atgm336h_data_t *src, crsf_telemetry_gps_t *dst)
{
    if (src == NULL || dst == NULL)
    {
        return;
    }

    dst->latitude = (int32_t)(src->latitude * 10000000.0);
    dst->longitude = (int32_t)(src->longitude * 10000000.0);
    dst->ground_speed = (uint16_t)(src->speed_kmh * 10.0f);
    dst->heading = (uint16_t)(src->course * 100.0f);
    dst->altitude_m = (uint16_t)src->altitude;
    dst->satellites = src->satellites;
}
