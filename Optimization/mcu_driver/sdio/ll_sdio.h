#ifndef __LL_SDIO_H
#define __LL_SDIO_H

#include "stdint.h"

typedef void (*sd_delay_ms_func_t)(uint32_t ms) ;


void SDIO_Register_Delay( sd_delay_ms_func_t us_delay_callback) ; 
void sdio_sdcard_init(void) ; 

#endif
