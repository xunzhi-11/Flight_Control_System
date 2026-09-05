#ifndef __LL_DMA_SPI_H
#define __LL_DMA_SPI_H

#include "stdint.h"

void DMA_SPI_RegisterTCCallback(uint8_t spi_id, void (*callback)(void)) ; 
void dma_to_spi1_rx_init(void) ; 
void dma_to_spi2_rx_init(void) ; 

void DMA_SPI1_TxRx_Trigger(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) ; 
void DMA_SPI2_TxRx_Trigger(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) ;

#endif
