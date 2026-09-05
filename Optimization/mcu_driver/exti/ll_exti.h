#ifndef __LL_EXTI_H
#define __LL_EXTI_H

#include "stdint.h"

void EXTI_RegisterCallback(uint32_t exti_line_0_to_15, void (*callback)(void))  ; 
void exti_bmm150_drdy_config(void) ; 
void EXTI_BMM150_Start(void) ; 
void exti_icm42688_drdy_config(void) ; 
void EXTI_ICM42688_Start(void) ; 
void exti_bmp390l_drdy_config(void) ; 
void EXTI_BMP390L_Start(void) ; 

#endif
