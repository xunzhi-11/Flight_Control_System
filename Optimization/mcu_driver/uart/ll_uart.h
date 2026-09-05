#ifndef __LL_UART_H
#define __LL_UART_H

#include "stdint.h"

uint8_t* UART2_Get_DMA_ReadBuffer_Addr(void) ; 
uint8_t* UART2_Get_DMA_WriteBuffer_Addr(void) ; 
uint16_t UART2_Get_DMA_Buffer_Size(void) ; 
uint8_t UART2_Update_Byte(uint8_t * out_put) ; 
void UART2_TX_Lockoff(void) ; 
uint8_t UART2_Transmit_DMA_Async(const uint8_t *data, uint16_t len) ; 

uint8_t* UART3_Get_DMA_ReadBuffer_Addr(void) ;
uint16_t UART3_Get_DMA_Buffer_Size(void) ;
uint16_t UART3_Get_Available_Byte_Count(void) ;
uint8_t UART3_Update_Byte(uint8_t *out_put) ;
uint16_t UART3_Read_Bytes(uint8_t *dst, uint16_t max_len) ;

void UART_Config_Init(void) ; 

#endif
