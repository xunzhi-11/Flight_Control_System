#ifndef __UART1_RINGBUF_H
#define __UART1_RINGBUF_H

#include "stdio.h"

uint8_t* UART1_Get_DMA_ReadBuffer_Addr(void) ;
uint8_t* UART1_Get_DMA_WriteBuffer_Addr(void) ; 
uint16_t UART1_Get_DMA_Buffer_Size(void) ; 
uint8_t UART1_Update_Byte(uint8_t * out_put) ; 
void UART1_TX_Lockoff(void) ; 
uint8_t UART1_Transmit_DMA_Async(const uint8_t *data, uint16_t len) ; 


#endif
