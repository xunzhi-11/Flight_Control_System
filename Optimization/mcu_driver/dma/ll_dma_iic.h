#ifndef __LL_DMA_IIC_H
#define __LL_DMA_IIC_H

#include "stdint.h"

void DMA_IIC_RegisterTCCallback(uint8_t iic_id, void (*callback)(void)) ; 
void dma_to_iic2_rx_init(void) ; 
void DMA_IIC2_Rx_Trigger(uint8_t *rx_buf, uint16_t len) ; 

#endif