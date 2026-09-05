#ifndef PID_TRACE_PROTOCOL_H
#define PID_TRACE_PROTOCOL_H

#include "stdint.h"

/* Frame magic 'PT' — keep in sync with tools/pid_trace_protocol.py */
#define PIDTRACE_FRAME_MAGIC     0x5054U
#define PIDTRACE_FRAME_SIZE      32U
#define PIDTRACE_DUMP_MAGIC      0x50444D50U /* 'PDMP' little-endian on wire */

#define PIDTRACE_FLAG_ARMED      0x01U
#define PIDTRACE_FLAG_OVERFLOW   0x02U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t seq;
    uint32_t ts_us;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t roll;
    int16_t pitch;
    int16_t mix_roll;
    int16_t mix_pitch;
    int16_t mix_yaw;
    uint8_t motor[4];
    uint8_t throttle_u8;
    uint8_t flags;
    uint16_t reserved;
} PidTrace_Frame_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame_count;
    uint16_t frame_bytes;
    uint16_t decim_ratio;
    uint32_t reserved;
} PidTrace_DumpHeader_t;

#endif /* PID_TRACE_PROTOCOL_H */
