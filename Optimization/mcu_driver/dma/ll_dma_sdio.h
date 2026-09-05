#ifndef __LL_DMA_SDIO_H
#define __LL_DMA_SDIO_H

#include "sys_software_config.h"

void dma_to_sdio_init(void) ;
void dma_sdio_SetState(SDIO_DMA_State_e state) ;
SDIO_DMA_State_e dma_sdio_GetState(void) ;
void dma_sdio_ClearState(void) ;


#endif
