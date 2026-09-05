#include "dma/ll_dma_uart.h"
#include "uart/ll_uart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "sys_hardware_config.h"

void dma_for_uart2_init(void)
{
    // DMA搬运接收数据初始化 (USART2_RX -> DMA1_Stream5_Channel4)
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    
    // 清除 USART2 残留标志
    (void)HW_UART_ELRS_INSTANCE->DR;
    LL_USART_ClearFlag_ORE(HW_UART_ELRS_INSTANCE);
    LL_USART_ClearFlag_FE(HW_UART_ELRS_INSTANCE);
    LL_USART_ClearFlag_NE(HW_UART_ELRS_INSTANCE);
    
    LL_DMA_InitTypeDef DMA_FOR_UART2_RX_InitStructure = {0};
    DMA_FOR_UART2_RX_InitStructure.Channel = HW_DMA_UART2_RX_CHANNEL;
    DMA_FOR_UART2_RX_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_FOR_UART2_RX_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA_FOR_UART2_RX_InitStructure.MemoryOrM2MDstAddress = (uint32_t)UART2_Get_DMA_ReadBuffer_Addr();
    DMA_FOR_UART2_RX_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_FOR_UART2_RX_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_FOR_UART2_RX_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_FOR_UART2_RX_InitStructure.NbData = UART2_Get_DMA_Buffer_Size();
    DMA_FOR_UART2_RX_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    DMA_FOR_UART2_RX_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&HW_UART_ELRS_INSTANCE->DR;
    DMA_FOR_UART2_RX_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_FOR_UART2_RX_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_FOR_UART2_RX_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    LL_DMA_Init(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM, &DMA_FOR_UART2_RX_InitStructure);
    
    // 禁用所有 DMA 中断
    LL_DMA_DisableIT_TC(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
    LL_DMA_DisableIT_HT(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
    LL_DMA_DisableIT_TE(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
    LL_DMA_DisableIT_DME(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
    LL_DMA_DisableIT_FE(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);

    LL_DMA_ClearFlag_TC5(HW_DMA_UART2_RX_INSTANCE);
    LL_DMA_ClearFlag_HT5(HW_DMA_UART2_RX_INSTANCE);
    LL_DMA_ClearFlag_TE5(HW_DMA_UART2_RX_INSTANCE);
    LL_DMA_ClearFlag_DME5(HW_DMA_UART2_RX_INSTANCE);
    LL_DMA_ClearFlag_FE5(HW_DMA_UART2_RX_INSTANCE);
    
    // 先使能 DMA 流，后使能 USART DMA 请求
    LL_DMA_EnableStream(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
    LL_USART_EnableDMAReq_RX(HW_UART_ELRS_INSTANCE);


    // DMA搬运发送数据初始化 (USART2_TX -> DMA1_Stream6_Channel4)
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1); 
    
    // 清除 USART2 残留标志
    (void)HW_UART_ELRS_INSTANCE->DR;
    LL_USART_ClearFlag_ORE(HW_UART_ELRS_INSTANCE);
    LL_USART_ClearFlag_FE(HW_UART_ELRS_INSTANCE);
    LL_USART_ClearFlag_NE(HW_UART_ELRS_INSTANCE);
    
    LL_DMA_InitTypeDef DMA_FOR_UART2_TX_InitStructure = {0};
    DMA_FOR_UART2_TX_InitStructure.Channel = HW_DMA_UART2_TX_CHANNEL;
    DMA_FOR_UART2_TX_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_FOR_UART2_TX_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA_FOR_UART2_TX_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_FOR_UART2_TX_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_FOR_UART2_TX_InitStructure.Mode = LL_DMA_MODE_NORMAL;
    DMA_FOR_UART2_TX_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    DMA_FOR_UART2_TX_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&USART2->DR;
    DMA_FOR_UART2_TX_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_FOR_UART2_TX_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_FOR_UART2_TX_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    LL_DMA_Init(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM, &DMA_FOR_UART2_TX_InitStructure);
    
    // 禁用部分 DMA 中断
    LL_DMA_DisableIT_HT(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM);
    LL_DMA_DisableIT_TE(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM);
    LL_DMA_DisableIT_DME(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM);
    LL_DMA_DisableIT_FE(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM);
    
    // 使能传输完成中断 (TC), 但不使能DMA流
    LL_DMA_EnableIT_TC(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM);
    // 使能 USART DMA TX 请求
    LL_USART_EnableDMAReq_TX(HW_UART_ELRS_INSTANCE);
}

void dma_for_uart3_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    
    // 清除 USART3 残留标志
    (void)HW_UART_GPS_INSTANCE->DR;
    LL_USART_ClearFlag_ORE(HW_UART_GPS_INSTANCE);
    LL_USART_ClearFlag_FE(HW_UART_GPS_INSTANCE);
    LL_USART_ClearFlag_NE(HW_UART_GPS_INSTANCE);
    
    LL_DMA_InitTypeDef DMA_FOR_UART3_RX_InitStructure = {0};
    DMA_FOR_UART3_RX_InitStructure.Channel = HW_DMA_UART3_RX_CHANNEL;
    DMA_FOR_UART3_RX_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_FOR_UART3_RX_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA_FOR_UART3_RX_InitStructure.MemoryOrM2MDstAddress = (uint32_t)UART3_Get_DMA_ReadBuffer_Addr();
    DMA_FOR_UART3_RX_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_FOR_UART3_RX_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_FOR_UART3_RX_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_FOR_UART3_RX_InitStructure.NbData = UART3_Get_DMA_Buffer_Size();
    DMA_FOR_UART3_RX_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    DMA_FOR_UART3_RX_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&HW_UART_GPS_INSTANCE->DR;
    DMA_FOR_UART3_RX_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_FOR_UART3_RX_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_FOR_UART3_RX_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    LL_DMA_Init(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM, &DMA_FOR_UART3_RX_InitStructure);
    
    // 禁用所有 DMA 中断
    LL_DMA_DisableIT_TC(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);
    LL_DMA_DisableIT_HT(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);
    LL_DMA_DisableIT_TE(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);
    LL_DMA_DisableIT_DME(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);
    LL_DMA_DisableIT_FE(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);

    LL_DMA_ClearFlag_TC1(HW_DMA_UART3_RX_INSTANCE);
    LL_DMA_ClearFlag_HT1(HW_DMA_UART3_RX_INSTANCE);
    LL_DMA_ClearFlag_TE1(HW_DMA_UART3_RX_INSTANCE);
    LL_DMA_ClearFlag_DME1(HW_DMA_UART3_RX_INSTANCE);
    LL_DMA_ClearFlag_FE1(HW_DMA_UART3_RX_INSTANCE);
    
    // 先使能 DMA 流，后使能 USART DMA 请求
    LL_DMA_EnableStream(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);
    LL_USART_EnableDMAReq_RX(HW_UART_GPS_INSTANCE);
}

void DMA1_Stream6_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC6(HW_DMA_UART2_TX_INSTANCE)) 
    {
        LL_DMA_ClearFlag_TC6(HW_DMA_UART2_TX_INSTANCE);   
        UART2_TX_Lockoff();            
    }
}
