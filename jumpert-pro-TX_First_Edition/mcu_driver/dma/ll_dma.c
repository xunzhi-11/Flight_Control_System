#include "dma/ll_dma.h"
#include "system_hardware_config.h"
#include "uart_ringbuf/uart1_ringbuf.h"
#include "app_task/app_task_joystick_update.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_adc.h"

static DMA_Event_Cb_t g_adc_dma_cb = NULL;

void DMA_FOR_UART1_Init(void)
{
    //dma搬运接收数据初始化
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    
    // 清除 USART1 残留标志
    (void)USART1->DR;
    LL_USART_ClearFlag_ORE(USART1);
    LL_USART_ClearFlag_FE(USART1);
    LL_USART_ClearFlag_NE(USART1);
    
    LL_DMA_InitTypeDef DMA_FOR_UART1_RX_InitStructure = {0};
    DMA_FOR_UART1_RX_InitStructure.Channel = HW_DMA_UART1_TX_CHANNEL;
    DMA_FOR_UART1_RX_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_FOR_UART1_RX_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA_FOR_UART1_RX_InitStructure.MemoryOrM2MDstAddress = (uint32_t)UART1_Get_DMA_ReadBuffer_Addr();
    DMA_FOR_UART1_RX_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_FOR_UART1_RX_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_FOR_UART1_RX_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
    DMA_FOR_UART1_RX_InitStructure.NbData = UART1_Get_DMA_Buffer_Size();
    DMA_FOR_UART1_RX_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    DMA_FOR_UART1_RX_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&USART1->DR;
    DMA_FOR_UART1_RX_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_FOR_UART1_RX_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_FOR_UART1_RX_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    LL_DMA_Init(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM, &DMA_FOR_UART1_RX_InitStructure);
    
    // 禁用所有 DMA 中断
    LL_DMA_DisableIT_TC(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    LL_DMA_DisableIT_HT(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    LL_DMA_DisableIT_TE(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    LL_DMA_DisableIT_DME(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    LL_DMA_DisableIT_FE(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    
    // 先使能 DMA 流，后使能 USART DMA 请求
    LL_DMA_EnableStream(HW_DMA_UART1_RX_INSTANCE, HW_DMA_UART1_RX_STREAM);
    LL_USART_EnableDMAReq_RX(USART1);


    // dma搬运发送数据初始化
     LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    
    // 清除 USART1 残留标志
    (void)USART1->DR;
    LL_USART_ClearFlag_ORE(USART1);
    LL_USART_ClearFlag_FE(USART1);
    LL_USART_ClearFlag_NE(USART1);
    
    LL_DMA_InitTypeDef DMA_FOR_UART1_TX_InitStructure = {0};
    DMA_FOR_UART1_TX_InitStructure.Channel = HW_DMA_UART1_TX_CHANNEL;
    DMA_FOR_UART1_TX_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_FOR_UART1_TX_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA_FOR_UART1_TX_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_FOR_UART1_TX_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_FOR_UART1_TX_InitStructure.Mode = LL_DMA_MODE_NORMAL;
    DMA_FOR_UART1_TX_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    DMA_FOR_UART1_TX_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&USART1->DR;
    DMA_FOR_UART1_TX_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_FOR_UART1_TX_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_FOR_UART1_TX_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    LL_DMA_Init(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM, &DMA_FOR_UART1_TX_InitStructure);
    
    // 禁用所有 DMA 中断
    LL_DMA_DisableIT_HT(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM);
    LL_DMA_DisableIT_TE(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM);
    LL_DMA_DisableIT_DME(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM);
    LL_DMA_DisableIT_FE(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM);
    
    // 使能 USART DMA 请求、传输完成中断,但不使能dma流
    LL_DMA_EnableIT_TC(HW_DMA_UART1_TX_INSTANCE, HW_DMA_UART1_TX_STREAM);
    LL_USART_EnableDMAReq_TX(USART1);

}

void DMA_FOR_ADC1_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    LL_DMA_InitTypeDef dma_initstructure = {0}; 

    dma_initstructure.MemoryOrM2MDstAddress  = (uint32_t)joystick_get_rawbuf_addr();
    dma_initstructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD; 
    dma_initstructure.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;    
    dma_initstructure.PeriphOrM2MSrcAddress  = LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA); 
    dma_initstructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD; 
    dma_initstructure.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;  
    dma_initstructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY; 
    dma_initstructure.Channel   = LL_DMA_CHANNEL_0;                  
    dma_initstructure.Mode      = LL_DMA_MODE_CIRCULAR;           
    dma_initstructure.Priority  = LL_DMA_PRIORITY_HIGH;              
    dma_initstructure.NbData    = 8;                                
    dma_initstructure.FIFOMode      = LL_DMA_FIFOMODE_DISABLE;
    dma_initstructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
    dma_initstructure.MemBurst      = LL_DMA_MBURST_SINGLE;
    dma_initstructure.PeriphBurst   = LL_DMA_PBURST_SINGLE;

    LL_DMA_Init(HW_DMA_ADC1_INSTANCE, HW_DMA_ADC1_STREAM, &dma_initstructure); 

    LL_DMA_EnableIT_HT(HW_DMA_ADC1_INSTANCE, HW_DMA_ADC1_STREAM); // Half Transfer
    LL_DMA_EnableIT_TC(HW_DMA_ADC1_INSTANCE, HW_DMA_ADC1_STREAM); // Transfer Complete 

    LL_DMA_EnableStream(HW_DMA_ADC1_INSTANCE, HW_DMA_ADC1_STREAM);
}

void DMA_FOR_OLED_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_DMA_DisableStream(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM);
    LL_DMA_SetChannelSelection(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, HW_DMA_OLED_I2C_TX_CHANNEL);
    LL_DMA_SetDataTransferDirection(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, (uint32_t)&(I2C1->DR)); // 目标地址为I2C数据寄存器
}



void DMA2_Stream7_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC7(DMA2)) 
    {
        LL_DMA_ClearFlag_TC7(DMA2);   
        
        while(!LL_USART_IsActiveFlag_TC(USART1));
        LL_USART_DisableDirectionTx(USART1);
        LL_USART_EnableDirectionRx(USART1);

        UART1_TX_Lockoff();            
    }
}


void DMA_ADC_RegisterCallback(DMA_Event_Cb_t cb) 
{
    g_adc_dma_cb = cb;
}

void DMA2_Stream4_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_HT4(DMA2)) {
        LL_DMA_ClearFlag_HT4(DMA2);
        if(g_adc_dma_cb) g_adc_dma_cb(0);
    }
    if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
        LL_DMA_ClearFlag_TC4(DMA2);
        if(g_adc_dma_cb) g_adc_dma_cb(1);
    }
}