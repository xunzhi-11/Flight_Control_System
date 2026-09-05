#ifndef __JOYSTICK_BUF_H
#define __JOYSTICK_BUF_H

#include "stdio.h"

uint16_t* joystick_get_rawbuf_addr(void) ; 
void Joystick_Get_Latest_Raw(uint16_t raw[4]) ;
void Task_Joystick_Init(void) ; 



#endif
