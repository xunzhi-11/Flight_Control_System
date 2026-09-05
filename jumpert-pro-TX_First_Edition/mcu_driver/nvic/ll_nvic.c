#include "nvic/ll_nvic.h"
#include "system_hardware_config.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_exti.h"

void NVIC_UART1_Config(void)
{
    NVIC_SetPriority(HW_IRQ_UART1, 5) ;   
    NVIC_EnableIRQ(HW_IRQ_UART1) ;        // 使能 USART1 中断

    NVIC_SetPriority(HW_IRQ_DMA_UART1_TX, 1) ; 
    NVIC_EnableIRQ(HW_IRQ_DMA_UART1_TX) ;  //使能传输完成中断
}

void NVIC_TIM3_Config(void)
{
    /*
     * TIM3 ISR uses FreeRTOS FromISR API (vTaskNotifyGiveFromISR),
     * so its preemption priority must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
     */
    NVIC_SetPriority(HW_IRQ_TIM3, 5) ;   
    NVIC_EnableIRQ(HW_IRQ_TIM3) ;
}

void NVIC_ADC1_Config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_ADC1, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)) ;
    NVIC_EnableIRQ(HW_IRQ_DMA_ADC1) ;
}