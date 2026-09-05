#ifndef __LL_DMA_H
#define __LL_DMA_H

#include "stdint.h"

typedef void (*DMA_Event_Cb_t)(uint8_t is_full);

void DMA_FOR_UART1_Init(void); 
void DMA_FOR_ADC1_Init(void); 
void DMA_FOR_OLED_Init(void); 
void DMA_ADC_RegisterCallback(DMA_Event_Cb_t cb) ; 

#endif
