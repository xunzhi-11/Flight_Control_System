#include "uart/ll_uart.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"
#include "stdio.h"

#ifdef __GNUC__
    int _write(int file, char *ptr, int len)
    {
        for(int i = 0 ; i < len ; i++)
        {
            while(!LL_USART_IsActiveFlag_TXE(HW_UART_DBG_INSTANCE)) ;
            LL_USART_TransmitData8(HW_UART_DBG_INSTANCE, (uint8_t)ptr[i]) ;
        }
        return len ;
    }
#endif

__attribute__((weak)) void UART1_IDEL_CallBack(void)
{

}

void UART_Config_Init(void)
{
    //UART1 Config
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1) ;

    LL_USART_Disable(HW_UART_RF_INSTANCE) ; 
    
    LL_USART_InitTypeDef USART1_InitStructure = {0} ;
    USART1_InitStructure.BaudRate = HW_UART_RF_BAUDRATE ; 
    USART1_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B ;
    USART1_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE ;
    USART1_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16 ;
    USART1_InitStructure.Parity = LL_USART_PARITY_NONE ;
    USART1_InitStructure.StopBits = LL_USART_STOPBITS_1 ;
    USART1_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX ;
    
    LL_USART_Init(HW_UART_RF_INSTANCE , &USART1_InitStructure) ;
    LL_USART_ConfigAsyncMode(HW_UART_RF_INSTANCE) ; // 异步模式配置
    LL_USART_EnableHalfDuplex(HW_UART_RF_INSTANCE) ; 
    LL_USART_Enable(HW_UART_RF_INSTANCE) ;

    LL_USART_EnableIT_IDLE(HW_UART_RF_INSTANCE) ; //UART1开启空闲中断

    //Debug UART Config (USART2)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2) ; 

    LL_USART_Disable(HW_UART_DBG_INSTANCE) ;

    LL_USART_InitTypeDef UART_DBG_InitStructure = {0} ; 
    UART_DBG_InitStructure.BaudRate = HW_UART_DBG_BAUDRATE ;
    UART_DBG_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B ;
    UART_DBG_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE ;
    UART_DBG_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16 ;
    UART_DBG_InitStructure.Parity = LL_USART_PARITY_NONE ;
    UART_DBG_InitStructure.StopBits = LL_USART_STOPBITS_1 ;
    UART_DBG_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX ;

    LL_USART_Init(HW_UART_DBG_INSTANCE , &UART_DBG_InitStructure) ; 

    LL_USART_ConfigAsyncMode(HW_UART_DBG_INSTANCE) ; // 异步模式配置
    LL_USART_Enable(HW_UART_DBG_INSTANCE) ;


}

void USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(HW_UART_RF_INSTANCE))
    {
        LL_USART_ClearFlag_IDLE(HW_UART_RF_INSTANCE) ;
        UART1_IDEL_CallBack() ; 
    }
}