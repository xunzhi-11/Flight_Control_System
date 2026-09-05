#include "sdio/ll_sdio.h"
#include "stdint.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

static sd_delay_ms_func_t s_sd_delay_ms = NULL ;

void SDIO_Register_Delay( sd_delay_ms_func_t ms_delay_callback)
{
    s_sd_delay_ms = ms_delay_callback;
}

static void sdio_delay_ms(uint32_t ms)
{
     if(s_sd_delay_ms != NULL)
    {
        s_sd_delay_ms(ms) ; 
    }
    else
    {
        uint32_t i = 1000000 ; 
        while(i--) ; 
    }
}

void sdio_sdcard_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SDIO);
    LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SDIO);
    LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SDIO);
    SDIO->POWER = 0x00;
    sdio_delay_ms(10); 
    
    SDIO->POWER = SDIO_POWER_PWRCTRL;
    sdio_delay_ms(2);
   
    SDIO->CLKCR = 0;
    SDIO->POWER = 0;
    SDIO->CMD = 0;
    SDIO->ARG = 0;
    SDIO->DTIMER = 0;
    SDIO->DLEN = 0;
    SDIO->DCTRL = 0;
    SDIO->ICR = 0x7FF;  // 清除所有标志
    
    /* 开启电源 */
    SDIO->POWER = SDIO_POWER_PWRCTRL;
    sdio_delay_ms(2) ; 
    
    /* 配置时钟 */
     SDIO->CLKCR = (SDIO_INIT_CLK_DIV << SDIO_CLKCR_CLKDIV_Pos) | 
                  SDIO_CLKCR_CLKEN ; 
    sdio_delay_ms(2) ; 
}