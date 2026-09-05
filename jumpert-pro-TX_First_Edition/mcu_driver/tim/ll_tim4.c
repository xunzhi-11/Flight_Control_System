#include "tim/ll_tim4.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_tim.h"


void TIM4_Config(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4) ; 

    LL_TIM_InitTypeDef tim4_initstructure = {0} ; 

    tim4_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1 ;
    tim4_initstructure.Prescaler = HW_TIM_ADC_TRIG_PSC ;
    tim4_initstructure.Autoreload = HW_TIM_ADC_TRIG_ARR ;
    tim4_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP ;

    LL_TIM_Init(HW_TIM_ADC_TRIG_INSTANCE , &tim4_initstructure) ;

    LL_TIM_OC_InitTypeDef tim4_oc_initstructure = {0} ;
    tim4_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1 ;     // PWM1 模式
    tim4_oc_initstructure.OCState = LL_TIM_OCSTATE_ENABLE ; // 使能通道
    tim4_oc_initstructure.CompareValue = 1000 ;             // 在 1000 的位置产生翻转 (50%占空比)
    LL_TIM_OC_Init(HW_TIM_ADC_TRIG_INSTANCE, HW_TIM_ADC_TRIG_CH, &tim4_oc_initstructure) ;
    // 使能 CH4 的预装载
    LL_TIM_OC_EnablePreload(HW_TIM_ADC_TRIG_INSTANCE, HW_TIM_ADC_TRIG_CH) ;

    LL_TIM_EnableCounter(HW_TIM_ADC_TRIG_INSTANCE) ; 
}