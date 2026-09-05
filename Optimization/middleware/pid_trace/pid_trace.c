#include "pid_trace/pid_trace.h"
#include "pid_trace/pid_trace_port.h"
#include "pid_trace/pid_trace_protocol.h"
#include "stddef.h"

#if PIDTRACE_ENABLE

#if PIDTRACE_PORT_USART1
#include "pid_trace/pid_trace_port_usart1.h"
#endif

#if PIDTRACE_TRIGGER_PA0
#include "pid_trace/pid_trace_trigger_pa0.h"
#endif

#include "stm32f4xx_ll_bus.h"

static const PidTrace_ExportSink_t *s_export_sink;

__attribute__((section(".ccmram"), aligned(4)))
static uint8_t s_buffer[PIDTRACE_BUFFER_BYTES];

static volatile uint32_t s_write_offset;
static volatile uint32_t s_frame_count;
static volatile uint16_t s_seq;
static volatile uint8_t s_recording;
static volatile uint8_t s_overflow;
static volatile uint8_t s_dumping;
static volatile uint32_t s_drop_count;
static uint8_t s_decim_left;
static uint8_t s_trigger_prev;
static uint8_t s_trigger_hold_ticks;
static uint8_t s_trigger_dumped_hold;
static uint32_t s_last_dump_frames;

#define PIDTRACE_TRIGGER_HOLD_TICKS 5U  /* 5 x 10ms service = 50ms hold */

static int16_t float_to_i16_scaled(float value, float scale)
{
    float scaled = value * scale;
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

static uint8_t float_to_u8_norm(float norm)
{
    if (norm <= 0.0f) {
        return 0U;
    }
    if (norm >= 1.0f) {
        return 255U;
    }
    return (uint8_t)(norm * 255.0f + 0.5f);
}

static uint8_t append_frame(const PidTrace_Frame_t *frame)
{
    const uint32_t need = (uint32_t)sizeof(PidTrace_Frame_t);

    if (s_write_offset + need > PIDTRACE_BUFFER_BYTES) {
        s_overflow = 1U;
        return 0U;
    }

    for (uint32_t i = 0U; i < need; i++) {
        s_buffer[s_write_offset + i] = ((const uint8_t *)frame)[i];
    }

    s_write_offset += need;
    s_frame_count++;
    return 1U;
}

static void pack_and_record(const PidTrace_Sample_t *sample)
{
    PidTrace_Frame_t frame;

    frame.magic = PIDTRACE_FRAME_MAGIC;
    frame.seq = s_seq++;
    frame.ts_us = sample->ts_us;
    frame.gx = float_to_i16_scaled(sample->gx_deg_s, 100.0f);
    frame.gy = float_to_i16_scaled(sample->gy_deg_s, 100.0f);
    frame.gz = float_to_i16_scaled(sample->gz_deg_s, 100.0f);
    frame.roll = float_to_i16_scaled(sample->roll_deg, 100.0f);
    frame.pitch = float_to_i16_scaled(sample->pitch_deg, 100.0f);
    frame.mix_roll = float_to_i16_scaled(sample->mix_roll, 1000.0f);
    frame.mix_pitch = float_to_i16_scaled(sample->mix_pitch, 1000.0f);
    frame.mix_yaw = float_to_i16_scaled(sample->mix_yaw, 1000.0f);
    frame.motor[0] = float_to_u8_norm(sample->motor[0]);
    frame.motor[1] = float_to_u8_norm(sample->motor[1]);
    frame.motor[2] = float_to_u8_norm(sample->motor[2]);
    frame.motor[3] = float_to_u8_norm(sample->motor[3]);
    frame.throttle_u8 = float_to_u8_norm(sample->throttle_norm);
    frame.flags = sample->armed ? PIDTRACE_FLAG_ARMED : 0U;
    if (s_overflow) {
        frame.flags |= PIDTRACE_FLAG_OVERFLOW;
    }
    frame.reserved = 0U;

    if (!append_frame(&frame)) {
        s_drop_count++;
    }
}

void PidTrace_RegisterExportSink(const PidTrace_ExportSink_t *sink)
{
    s_export_sink = sink;
}

void PidTrace_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CCMDATARAM);

    s_export_sink = NULL;
    s_write_offset = 0U;
    s_frame_count = 0U;
    s_seq = 0U;
    s_recording = 0U;
    s_overflow = 0U;
    s_dumping = 0U;
    s_drop_count = 0U;
    s_decim_left = 0U;
    s_trigger_prev = 0U;
    s_trigger_hold_ticks = 0U;
    s_trigger_dumped_hold = 0U;
    s_last_dump_frames = 0U;

#if PIDTRACE_TRIGGER_PA0
    PidTrace_PortTrigger_Init();
#endif

#if PIDTRACE_PORT_USART1
    PidTrace_PortUSART1_Register();
#endif
}

void PidTrace_Reset(void)
{
    s_write_offset = 0U;
    s_frame_count = 0U;
    s_seq = 0U;
    s_overflow = 0U;
    s_drop_count = 0U;
    s_decim_left = 0U;
}

static void start_dump(void)
{
    PidTrace_DumpHeader_t header;
    uint32_t offset = 0U;
    uint16_t chunk;
    uint32_t frames_to_send;

    if (s_export_sink == NULL || s_export_sink->write == NULL) {
        return;
    }

    if (s_dumping) {
        return;
    }

    frames_to_send = s_frame_count;
    s_last_dump_frames = frames_to_send;

    if (frames_to_send == 0U) {
        s_dumping = 1U;
        header.magic = PIDTRACE_DUMP_MAGIC;
        header.frame_count = 0U;
        header.frame_bytes = (uint16_t)sizeof(PidTrace_Frame_t);
        header.decim_ratio = (uint16_t)PIDTRACE_DECIM_RATIO;
        header.reserved = 0U;
        s_export_sink->write((const uint8_t *)&header, (uint16_t)sizeof(header));
        s_dumping = 0U;
        return;
    }

    s_dumping = 1U;

    header.magic = PIDTRACE_DUMP_MAGIC;
    header.frame_count = frames_to_send;
    header.frame_bytes = (uint16_t)sizeof(PidTrace_Frame_t);
    header.decim_ratio = (uint16_t)PIDTRACE_DECIM_RATIO;
    header.reserved = 0U;

    s_export_sink->write((const uint8_t *)&header, (uint16_t)sizeof(header));

    while (offset < s_write_offset) {
        chunk = (uint16_t)(s_write_offset - offset);
        if (chunk > 512U) {
            chunk = 512U;
        }
        s_export_sink->write(&s_buffer[offset], chunk);
        offset += chunk;
    }

    PidTrace_Reset();
    s_recording = 0U;
    s_dumping = 0U;
}

void PidTrace_Service(void)
{
    uint8_t pressed;

#if PIDTRACE_TRIGGER_PA0
    pressed = PidTrace_PortTrigger_IsPressed();
#else
    pressed = 0U;
#endif

    if (pressed) {
        if (!s_trigger_prev) {
            start_dump();
            s_trigger_dumped_hold = 1U;
            s_trigger_hold_ticks = PIDTRACE_TRIGGER_HOLD_TICKS;
        } else if (!s_trigger_dumped_hold && s_trigger_hold_ticks < PIDTRACE_TRIGGER_HOLD_TICKS) {
            s_trigger_hold_ticks++;
            if (s_trigger_hold_ticks >= PIDTRACE_TRIGGER_HOLD_TICKS) {
                start_dump();
                s_trigger_dumped_hold = 1U;
            }
        }
    } else {
        s_trigger_hold_ticks = 0U;
        s_trigger_dumped_hold = 0U;
    }

    s_trigger_prev = pressed;
}

uint8_t PidTrace_IsTriggerPressed(void)
{
#if PIDTRACE_TRIGGER_PA0
    return PidTrace_PortTrigger_IsPressed();
#else
    return 0U;
#endif
}

void PidTrace_Feed(const PidTrace_Sample_t *sample)
{
    if (sample == NULL || s_dumping) {
        return;
    }

    if (sample->throttle_norm >= PIDTRACE_THROTTLE_ARM_NORM) {
        s_recording = 1U;
    } else {
        s_recording = 0U;
        s_decim_left = 0U;
        return;
    }

    if (PIDTRACE_DECIM_RATIO > 1U) {
        if (s_decim_left > 0U) {
            s_decim_left--;
            return;
        }
        s_decim_left = (uint8_t)(PIDTRACE_DECIM_RATIO - 1U);
    }

    pack_and_record(sample);
}

uint8_t PidTrace_IsRecording(void)
{
    return s_recording;
}

uint8_t PidTrace_IsDumping(void)
{
    return s_dumping;
}

uint32_t PidTrace_GetFrameCount(void)
{
    return s_frame_count;
}

uint32_t PidTrace_GetDropCount(void)
{
    return s_drop_count;
}

uint32_t PidTrace_GetLastDumpFrames(void)
{
    return s_last_dump_frames;
}

#endif /* PIDTRACE_ENABLE */
