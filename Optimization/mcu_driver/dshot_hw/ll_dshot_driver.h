#ifndef __LL_DSHOT_DRIVER_H
#define __LL_DSHOT_DRIVER_H

#include "stdint.h"

uint16_t* HW_DShot_Get_Buffer_Addr(void) ; 
void dshot_trigger(uint16_t m1_pkt, uint16_t m2_pkt, uint16_t m3_pkt, uint16_t m4_pkt) ; 

#endif