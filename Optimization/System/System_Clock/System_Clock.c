#include "System_Clock/System_Clock.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_utils.h"

/* Clock debug probes (inspect in debugger; no IO required) */
#ifndef CLOCK_DIAG_ENABLE
#define CLOCK_DIAG_ENABLE 1
#endif

#if CLOCK_DIAG_ENABLE
volatile uint32_t g_clk_step = 0;
volatile uint8_t  g_clk_fail_code = 0;
volatile uint32_t g_clk_rcc_cr = 0;
volatile uint32_t g_clk_rcc_cfgr = 0;
volatile uint32_t g_clk_rcc_pllcfgr = 0;
volatile uint32_t g_clk_flash_acr = 0;
static inline void clk_snap(uint8_t fail_code, uint32_t step)
{
    g_clk_fail_code   = fail_code;
    g_clk_step        = step;
    g_clk_rcc_cr      = RCC->CR;
    g_clk_rcc_cfgr    = RCC->CFGR;
    g_clk_rcc_pllcfgr = RCC->PLLCFGR;
    g_clk_flash_acr   = FLASH->ACR;
}
#else
static inline void clk_snap(uint8_t fail_code, uint32_t step) { (void)fail_code; (void)step; }
#endif

uint8_t SystemClock_Config(void)
{
    uint32_t timeout;

    /* 先保证 HSI 运行，并切回 HSI，避免在 PLL 作为 SYSCLK 时去关 PLL 导致卡死 */
    clk_snap(0, 10);
    LL_RCC_HSI_Enable();
    timeout = 0x200000U;
    while ((LL_RCC_HSI_IsReady() != 1U) && (timeout-- != 0U)) {}
    if (LL_RCC_HSI_IsReady() != 1U) {
        clk_snap(11, 19);
        return 11;
    }

    clk_snap(0, 20);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    timeout = 0x200000U;
    while ((LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) && (timeout-- != 0U)) {}
    if (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
        clk_snap(12, 29);
        return 12;
    }

    /* 开启电源接口时钟并设置电压调节器 (168MHz 需要 Scale1) */
    clk_snap(0, 30);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* 配置 Flash 等待周期（先设高一点，避免提频时取指异常），并带超时确认 */
    clk_snap(0, 40);
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    timeout = 0x200000U;
    while ((LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) && (timeout-- != 0U)) {}
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {
        clk_snap(13, 49);
        return 13;
    }

    /* 缓存/预取：可开可不开，这里保留开启（不依赖是否切到 PLL） */
    clk_snap(0, 50);
    LL_FLASH_EnablePrefetch();
    LL_FLASH_EnableInstCache();
    LL_FLASH_EnableDataCache();

    /* 关闭 PLL（带超时，避免异常状态卡死） */
    clk_snap(0, 60);
    if (LL_RCC_PLL_IsReady() == 1U) {
        LL_RCC_PLL_Disable();
        timeout = 0x200000U;
        while ((LL_RCC_PLL_IsReady() == 1U) && (timeout-- != 0U)) {}
        if (LL_RCC_PLL_IsReady() == 1U) {
            clk_snap(14, 69);
            return 14;
        }
    }

    /* 配置总线分频（目标：HCLK=168, APB1=42, APB2=84） */
    clk_snap(0, 70);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

    /* 选择 PLL 输入源：优先 HSE(8MHz)，失败则回退 HSI(16MHz) */
    clk_snap(0, 80);
    LL_RCC_HSE_DisableBypass();
    LL_RCC_HSE_Enable();
    timeout = 0x200000U;
    while ((LL_RCC_HSE_IsReady() != 1U) && (timeout-- != 0U)) {}

    if (LL_RCC_HSE_IsReady() == 1U) {
        clk_snap(0, 81);
        /* 168MHz = (8MHz / 8) * 336 / 2 */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
        //为 SDIO/USB 配置 48MHz 时钟 (336 / 7 = 48)
        LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLQ_DIV_7);
    } else {
        clk_snap(0, 82);
        /* HSE 未就绪：关掉 HSE，继续用 HSI 作 PLL 输入 */
        LL_RCC_HSE_Disable();
        /* 168MHz = (16MHz / 16) * 336 / 2 */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_2);
        //同样为 SDIO/USB 配置 48MHz 时钟 (336 / 7 = 48)
        LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLQ_DIV_7);
    }

    /* 开启 PLL 并等待锁定（带超时） */
    clk_snap(0, 90);
    LL_RCC_PLL_Enable();
    timeout = 0x200000U;
    while ((LL_RCC_PLL_IsReady() != 1U) && (timeout-- != 0U)) {}
    if (LL_RCC_PLL_IsReady() != 1U) {
        clk_snap(15, 99);
        return 15;
    }

    /* 切换系统时钟到 PLL（带超时） */
    clk_snap(0, 100);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    timeout = 0x200000U;
    while ((LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) && (timeout-- != 0U)) {}
    if (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
        clk_snap(16, 109);
        return 16;
    }

    /* 按实际寄存器更新 SystemCoreClock，避免后续 delay/外设时基错误 */
    clk_snap(0, 110);
    SystemCoreClockUpdate();

    clk_snap(0, 120);
    return 1;
}

void System_Set1msTick(uint64_t Onems_Tick)
{
    LL_Init1msTick(Onems_Tick); 
}
