#ifndef PID_TRACE_CONFIG_H
#define PID_TRACE_CONFIG_H

#include "stdint.h"
#include "vib_log/vib_log_config.h"

#ifndef PIDTRACE_ENABLE
#define PIDTRACE_ENABLE 0
#endif

#if PIDTRACE_ENABLE && VIBLOG_ENABLE
#error "PIDTRACE and VIBLOG both use USART1; enable only one (set VIBLOG_ENABLE=0 or PIDTRACE_ENABLE=0)."
#endif

#if PIDTRACE_ENABLE

/* IMU/control path ~4 kHz; record every 4th sample => ~1 kHz. */
#ifndef PIDTRACE_DECIM_RATIO
#define PIDTRACE_DECIM_RATIO 4U
#endif

#ifndef PIDTRACE_THROTTLE_ARM_NORM
#define PIDTRACE_THROTTLE_ARM_NORM 0.40f
#endif

/* Ring buffer in CCM (bytes). Must fit frame size alignment. */
#ifndef PIDTRACE_BUFFER_BYTES
#define PIDTRACE_BUFFER_BYTES (63U * 1024U)
#endif

#ifndef PIDTRACE_PORT_USART1
#define PIDTRACE_PORT_USART1 1
#endif

#ifndef PIDTRACE_TRIGGER_PA0
#define PIDTRACE_TRIGGER_PA0 1
#endif

#endif /* PIDTRACE_ENABLE */

#endif /* PID_TRACE_CONFIG_H */
