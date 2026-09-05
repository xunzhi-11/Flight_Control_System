#ifndef __LL_IIC_H
#define __LL_IIC_H

#include "stdbool.h"
#include "stdint.h"

void OLED_IIC_Init(void) ; 
void HW_I2C_WriteCmd_Blocking(uint8_t slave_addr, uint8_t cmd) ;
void HW_I2C_DMA_Trigger(uint8_t slave_addr, uint8_t *data, uint16_t len) ;
bool HW_I2C_DMA_Check_And_Stop(void) ;

#endif