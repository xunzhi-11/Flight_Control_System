#include "exti/ll_exti.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

static void (*s_exti_callbacks[16])(void) = {NULL};

// 对外暴露的注册接口
void EXTI_RegisterCallback(uint32_t exti_line_0_to_15, void (*callback)(void)) 
{
    if (exti_line_0_to_15 < 16) 
    {
        s_exti_callbacks[exti_line_0_to_15] = callback;
    }
}

void exti_bmm150_drdy_config(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG) ;
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA , LL_SYSCFG_EXTI_LINE1 ) ;
    
    LL_EXTI_InitTypeDef EXTI_InitStructure = {0} ;
    EXTI_InitStructure.LineCommand = ENABLE ;
    EXTI_InitStructure.Line_0_31 = HW_EXTI_BMM150_LINE ;
    EXTI_InitStructure.Mode = LL_EXTI_MODE_IT ;
    EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING ;
    LL_EXTI_Init(&EXTI_InitStructure) ;
  
    NVIC_SetPriority(HW_IRQ_EXTI_BMM150 , HW_EXTI_BMM150_IRQ_PRIORITY) ;
}

void EXTI_BMM150_Start(void)
{
    LL_EXTI_ClearFlag_0_31(HW_EXTI_BMM150_LINE);
    NVIC_EnableIRQ(HW_IRQ_EXTI_BMM150) ; 
}

void exti_icm42688_drdy_config(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC , LL_SYSCFG_EXTI_LINE0 );
    
    LL_EXTI_InitTypeDef EXTI_InitStructure = {0} ;
    EXTI_InitStructure.LineCommand = ENABLE;
    EXTI_InitStructure.Line_0_31 = HW_EXTI_ICM42688_LINE  ;
    EXTI_InitStructure.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStructure);

    NVIC_SetPriority(HW_IRQ_EXTI_ICM42688 , HW_EXTI_ICM42688_IRQ_PRIORITY);    
}

void EXTI_ICM42688_Start(void)
{
    LL_EXTI_ClearFlag_0_31(HW_EXTI_ICM42688_LINE);
    NVIC_EnableIRQ(HW_IRQ_EXTI_ICM42688) ; 
}

void exti_bmp390l_drdy_config(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB , LL_SYSCFG_EXTI_LINE5 );
    
    LL_EXTI_InitTypeDef EXTI_InitStructure = {0} ;
    EXTI_InitStructure.LineCommand = ENABLE;
    EXTI_InitStructure.Line_0_31 = HW_EXTI_BMP390L_LINE  ;
    EXTI_InitStructure.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStructure);

    NVIC_SetPriority(HW_IRQ_EXTI_BMP390L , HW_EXTI_BMP390L_IRQ_PRIORITY);    
}

void EXTI_BMP390L_Start(void)
{
    LL_EXTI_ClearFlag_0_31(HW_EXTI_BMP390L_LINE);
    NVIC_EnableIRQ(HW_IRQ_EXTI_BMP390L) ; 
}

void EXTI1_IRQHandler(void) 
{
    EXTI->PR = HW_EXTI_BMM150_LINE; 
    s_exti_callbacks[1]();
}


void EXTI0_IRQHandler(void) 
{
    EXTI->PR = HW_EXTI_ICM42688_LINE; 
    s_exti_callbacks[0]();
}

void EXTI9_5_IRQHandler(void)
{
    EXTI->PR = HW_EXTI_BMP390L_LINE ; 
    s_exti_callbacks[5]() ;
}