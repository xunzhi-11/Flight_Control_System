#ifndef __SYSTEM_CLOCK_H
#define __SYSTEM_CLOCK_H

#include "stdint.h"

uint8_t SystemClock_Config(void) ; 
void System_Set1msTick(uint64_t Onems_Tick) ; 

#endif
