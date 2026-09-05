#ifndef PID_TRACE_PORT_H
#define PID_TRACE_PORT_H

#include "stdint.h"
#include "pid_trace/pid_trace_config.h"

#if PIDTRACE_ENABLE

/*
 * Burst export sink: blocking TX acceptable (called outside 4 kHz loop).
 * write(): send len bytes; return bytes sent.
 * is_busy(): optional back-pressure (0 = ready).
 */
typedef struct {
    uint16_t (*write)(const uint8_t *data, uint16_t len);
    uint8_t (*is_busy)(void);
} PidTrace_ExportSink_t;

void PidTrace_RegisterExportSink(const PidTrace_ExportSink_t *sink);

#endif /* PIDTRACE_ENABLE */

#endif /* PID_TRACE_PORT_H */
