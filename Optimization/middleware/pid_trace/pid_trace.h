#ifndef PID_TRACE_H
#define PID_TRACE_H

#include "stdint.h"
#include "pid_trace/pid_trace_config.h"

#if PIDTRACE_ENABLE

typedef struct {
    float gx_deg_s;
    float gy_deg_s;
    float gz_deg_s;
    float roll_deg;
    float pitch_deg;
    float mix_roll;
    float mix_pitch;
    float mix_yaw;
    float motor[4];
    float throttle_norm;
    uint8_t armed;
    uint32_t ts_us;
} PidTrace_Sample_t;

void PidTrace_Init(void);
void PidTrace_Reset(void);

void PidTrace_Feed(const PidTrace_Sample_t *sample);
void PidTrace_Service(void);

uint8_t PidTrace_IsTriggerPressed(void);

uint8_t PidTrace_IsRecording(void);
uint8_t PidTrace_IsDumping(void);
uint32_t PidTrace_GetFrameCount(void);
uint32_t PidTrace_GetDropCount(void);
uint32_t PidTrace_GetLastDumpFrames(void);

#else

typedef struct {
    float gx_deg_s;
    float gy_deg_s;
    float gz_deg_s;
    float roll_deg;
    float pitch_deg;
    float mix_roll;
    float mix_pitch;
    float mix_yaw;
    float motor[4];
    float throttle_norm;
    uint8_t armed;
    uint32_t ts_us;
} PidTrace_Sample_t;

static inline void PidTrace_Init(void) {}
static inline void PidTrace_Reset(void) {}
static inline void PidTrace_Feed(const PidTrace_Sample_t *sample) { (void)sample; }
static inline void PidTrace_Service(void) {}
static inline uint8_t PidTrace_IsTriggerPressed(void) { return 0U; }
static inline uint8_t PidTrace_IsRecording(void) { return 0U; }
static inline uint8_t PidTrace_IsDumping(void) { return 0U; }
static inline uint32_t PidTrace_GetFrameCount(void) { return 0U; }
static inline uint32_t PidTrace_GetDropCount(void) { return 0U; }
static inline uint32_t PidTrace_GetLastDumpFrames(void) { return 0U; }

#endif /* PIDTRACE_ENABLE */

#endif /* PID_TRACE_H */
