#include "vib_log/vib_log.h"
#include "vib_log/vib_log_port.h"
#include "vib_log/vib_log_protocol.h"
#include "stddef.h"

#if VIBLOG_ENABLE

#if VIBLOG_PORT_USART1
#include "vib_log/vib_log_port_usart1.h"
#endif

static const VibLog_Sink_t *s_sink;
static uint8_t s_enabled;
static uint8_t s_throttle_u8;
static uint8_t s_flags;
static uint16_t s_seq;
static uint8_t s_decim_left;
static uint32_t s_drop_count;

static int16_t viblog_float_to_gyro_i16(float deg_s)
{
    float scaled = deg_s * VIBLOG_GYRO_SCALE;
    if (scaled >= 32767.0f) {
        return 32767;
    }
    if (scaled <= -32768.0f) {
        return -32768;
    }
    if (scaled >= 0.0f) {
        return (int16_t)(scaled + 0.5f);
    }
    return (int16_t)(scaled - 0.5f);
}

void VibLog_RegisterSink(const VibLog_Sink_t *sink)
{
    s_sink = sink;
}

void VibLog_Init(void)
{
    s_sink = NULL;
    s_enabled = 0U;
    s_throttle_u8 = 0U;
    s_flags = 0U;
    s_seq = 0U;
    s_decim_left = 0U;
    s_drop_count = 0U;

#if VIBLOG_PORT_USART1
    VibLog_PortUSART1_Register();
#endif

#if VIBLOG_AUTO_ENABLE
    if (s_sink != NULL) {
        s_enabled = 1U;
    }
#endif
}

void VibLog_SetEnabled(uint8_t enabled)
{
    s_enabled = enabled ? 1U : 0U;
    if (!s_enabled) {
        s_decim_left = 0U;
    }
}

uint8_t VibLog_IsEnabled(void)
{
    return s_enabled;
}

void VibLog_SetMeta(uint8_t throttle_u8, uint8_t armed)
{
    s_throttle_u8 = throttle_u8;
    s_flags = armed ? VIBLOG_FLAG_ARMED : 0U;
}

void VibLog_Poll(void)
{
    if (s_sink != NULL && s_sink->poll != NULL) {
        s_sink->poll();
    }
}

static void viblog_emit_frame(float gx_deg_s, float gy_deg_s, float gz_deg_s, uint32_t ts_us)
{
    VibLog_Frame_t frame;

    if (s_sink == NULL || s_sink->write == NULL) {
        s_drop_count++;
        return;
    }

    frame.magic = VIBLOG_MAGIC;
    frame.seq = s_seq;
    frame.ts_us = ts_us;
    frame.gx = viblog_float_to_gyro_i16(gx_deg_s);
    frame.gy = viblog_float_to_gyro_i16(gy_deg_s);
    frame.gz = viblog_float_to_gyro_i16(gz_deg_s);
    frame.throttle = s_throttle_u8;
    frame.flags = s_flags;

    if (s_sink->write((const uint8_t *)&frame, (uint16_t)sizeof(frame)) == (uint16_t)sizeof(frame)) {
        s_seq++;
    } else {
        s_drop_count++;
    }

    VibLog_Poll();
}

void VibLog_FeedImu(float gx_deg_s, float gy_deg_s, float gz_deg_s, uint32_t ts_us)
{
    if (!s_enabled) {
        return;
    }

    if (VIBLOG_DECIM_RATIO > 1U) {
        if (s_decim_left > 0U) {
            s_decim_left--;
            VibLog_Poll();
            return;
        }
        s_decim_left = (uint8_t)(VIBLOG_DECIM_RATIO - 1U);
    }

    viblog_emit_frame(gx_deg_s, gy_deg_s, gz_deg_s, ts_us);
}

uint32_t VibLog_GetDropCount(void)
{
    return s_drop_count;
}

uint16_t VibLog_GetSeq(void)
{
    return s_seq;
}

#endif /* VIBLOG_ENABLE */
