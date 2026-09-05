#ifndef __BMP390L_H
#define __BMP390L_H


#include "data_structure_types.h"
#include "stdint.h"

void bmp390l_drdy_exti_handler(void)  ; 
void bmp390l_dma_rx_complete_callback(void) ; 
void BMP390l_Register_Delay(void (*delay_func)(uint32_t ms)) ; 
void bmp390l_init(void) ; 
void bmp390l_get_real_data(BAROMETER_Real_t* out_data) ; 

#endif
