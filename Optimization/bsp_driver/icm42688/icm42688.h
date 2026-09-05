#ifndef __ICM42688_H
#define __ICM42688_H

#include "stdint.h"
#include "data_structure_types.h"


void icm42688_init(void) ; 
void icm42688_drdy_exti_handler(void) ; 
void icm42688_get_rawdata(IMU_Raw_t * out_data) ; 
void icm42688_register_delay(void (*delay_func)(uint32_t ms)) ; 
void icm42688_ahrs_task_trigger_register(void (*callback)(void)) ; 

#endif
