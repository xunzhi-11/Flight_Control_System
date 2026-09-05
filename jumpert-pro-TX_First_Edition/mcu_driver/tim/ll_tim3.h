#ifndef __LL_TIM3_H
#define __LL_TIM3_H

typedef void (*tim3_callback_t)(void) ;

void TIM3_Config(void) ; 
void TIM3_RegisterCallback(tim3_callback_t cb) ;

#endif
