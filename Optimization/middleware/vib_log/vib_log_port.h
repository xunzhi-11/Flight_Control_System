#ifndef VIB_LOG_PORT_H
#define VIB_LOG_PORT_H

#include "stdint.h"
#include "vib_log/vib_log_config.h"

#if VIBLOG_ENABLE

/*
 * Transport sink (open-closed): core packs frames; platform provides non-blocking TX.
 * write(): copy up to len bytes; return bytes accepted (may be < len if buffer full).
 * poll():  drain pending bytes (call from high-rate path, e.g. IMU loop).
 */
typedef struct {
    uint16_t (*write)(const uint8_t *data, uint16_t len);
    void (*poll)(void);
} VibLog_Sink_t;

void VibLog_RegisterSink(const VibLog_Sink_t *sink);

#endif /* VIBLOG_ENABLE */

#endif /* VIB_LOG_PORT_H */
