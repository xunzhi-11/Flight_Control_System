#ifndef VIB_LOG_H
#define VIB_LOG_H

#include "stdint.h"
#include "vib_log/vib_log_config.h"

#if VIBLOG_ENABLE

void VibLog_Init(void);
void VibLog_SetEnabled(uint8_t enabled);
uint8_t VibLog_IsEnabled(void);

void VibLog_SetMeta(uint8_t throttle_u8, uint8_t armed);
void VibLog_FeedImu(float gx_deg_s, float gy_deg_s, float gz_deg_s, uint32_t ts_us);
void VibLog_Poll(void);

uint32_t VibLog_GetDropCount(void);
uint16_t VibLog_GetSeq(void);

#else

static inline void VibLog_Init(void) {}
static inline void VibLog_SetEnabled(uint8_t enabled) { (void)enabled; }
static inline uint8_t VibLog_IsEnabled(void) { return 0U; }
static inline void VibLog_SetMeta(uint8_t throttle_u8, uint8_t armed)
{
    (void)throttle_u8;
    (void)armed;
}
static inline void VibLog_FeedImu(float gx_deg_s, float gy_deg_s, float gz_deg_s, uint32_t ts_us)
{
    (void)gx_deg_s;
    (void)gy_deg_s;
    (void)gz_deg_s;
    (void)ts_us;
}
static inline void VibLog_Poll(void) {}
static inline uint32_t VibLog_GetDropCount(void) { return 0U; }
static inline uint16_t VibLog_GetSeq(void) { return 0U; }

#endif /* VIBLOG_ENABLE */

#endif /* VIB_LOG_H */
