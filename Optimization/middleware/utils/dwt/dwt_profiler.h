#ifndef DWT_PROFILER_H
#define DWT_PROFILER_H

#include <stdint.h>

#ifndef DWT_PROFILER_ENABLE
#define DWT_PROFILER_ENABLE 1
#endif

void DWT_Init(void);
void DWT_Profiler_Start(void);
void DWT_Profiler_MarkDshotComplete(void);

#if DWT_PROFILER_ENABLE
extern volatile uint32_t g_dwt_imu_to_dshot_cycles;
extern volatile float    g_dwt_imu_to_dshot_us;
extern volatile uint32_t g_dwt_sample_count;
#endif

#endif
