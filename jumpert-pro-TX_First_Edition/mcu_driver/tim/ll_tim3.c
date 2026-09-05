#include "tim/ll_tim3.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_tim.h"

static tim3_callback_t s_tim3_callback = NULL ;   

void TIM3_RegisterCallback(tim3_callback_t cb)
{
    s_tim3_callback = cb ;
}


void TIM3_Config(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3) ; 

    LL_TIM_InitTypeDef tim3_initstructure = {0} ; 

    tim3_initstructure.Autoreload = HW_TIM_RC_SCHED_ARR ;
    tim3_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1 ;
    tim3_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP ;
    tim3_initstructure.Prescaler = HW_TIM_RC_SCHED_PSC ;
    tim3_initstructure.RepetitionCounter = 1 ;

    LL_TIM_Init(HW_TIM_RC_SCHED_INSTANCE , &tim3_initstructure) ; 

    LL_TIM_EnableIT_UPDATE(HW_TIM_RC_SCHED_INSTANCE) ;
    
    LL_TIM_EnableCounter(HW_TIM_RC_SCHED_INSTANCE) ;
}


void TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(HW_TIM_RC_SCHED_INSTANCE))
    {
        LL_TIM_ClearFlag_UPDATE(HW_TIM_RC_SCHED_INSTANCE) ;
        if (s_tim3_callback) 
        {
        s_tim3_callback() ;  
        }
    }
}