#ifndef __BMM150_H
#define __BMM150_H

#include "stdint.h"
#include "data_structure_types.h"

void BMM150_Register_Delay(void (*delay_func)(uint32_t ms)) ; 
void bmm150_drdy_exti_handler(void) ; 
uint8_t BMM150_Get_ChipID(void) ; 
void bmm150_get_trimed_data(MAGNETOMETER_Trimed_t* out_data) ; 
void bmm150_init(void) ; 

#endif

