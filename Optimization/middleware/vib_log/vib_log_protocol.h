#ifndef VIB_LOG_PROTOCOL_H
#define VIB_LOG_PROTOCOL_H

#include "stdint.h"

/*
 * Binary frame layout (little-endian, 16 bytes, no padding).
 * Must stay in sync with tools/vib_protocol.py (FRAME_FMT = "<HHIhhhBB").
 */
#define VIBLOG_MAGIC           0x5642U
#define VIBLOG_FRAME_SIZE      16U
#define VIBLOG_FLAG_ARMED      0x01U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t seq;
    uint32_t ts_us;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint8_t throttle;
    uint8_t flags;
} VibLog_Frame_t;

#endif /* VIB_LOG_PROTOCOL_H */
