#ifndef PID_TRACE_TRIGGER_PA0_H
#define PID_TRACE_TRIGGER_PA0_H

#include "pid_trace/pid_trace_config.h"

#if PIDTRACE_ENABLE

void PidTrace_PortTrigger_Init(void);
uint8_t PidTrace_PortTrigger_IsPressed(void);

#endif /* PIDTRACE_ENABLE */

#endif /* PID_TRACE_TRIGGER_PA0_H */
