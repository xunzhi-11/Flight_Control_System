#include "utils/dwt/dwt_profiler.h"
#include "stm32f4xx.h"

#if DWT_PROFILER_ENABLE

volatile uint32_t g_dwt_imu_to_dshot_cycles = 0;
volatile float    g_dwt_imu_to_dshot_us = 0.0f;
volatile uint32_t g_dwt_sample_count = 0;

static volatile uint32_t s_start_cycles = 0;
static volatile uint8_t  s_pending = 0;

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Profiler_Start(void)
{
    s_start_cycles = DWT->CYCCNT;
    s_pending = 1;
}

void DWT_Profiler_MarkDshotComplete(void)
{
    if (!s_pending) {
        return;
    }

    uint32_t delta = DWT->CYCCNT - s_start_cycles;

    g_dwt_imu_to_dshot_cycles = delta;
    g_dwt_imu_to_dshot_us = (float)delta * 1000000.0f / (float)SystemCoreClock;
    g_dwt_sample_count++;
    s_pending = 0;
}

#else

void DWT_Init(void) {}
void DWT_Profiler_Start(void) {}
void DWT_Profiler_MarkDshotComplete(void) {}

#endif
