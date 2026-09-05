#include "dma/ll_dma_iic.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_i2c.h"
#include "sys_hardware_config.h"
#include <stddef.h>

static void (*s_dma_iic_tc_callbacks[3])(void) = {NULL, NULL, NULL} ; 

// 注册接口
void DMA_IIC_RegisterTCCallback(uint8_t iic_id, void (*callback)(void)) 
{
    if (iic_id < 3) 
    {
        s_dma_iic_tc_callbacks[iic_id] = callback ;
    }
}

void dma_to_iic2_rx_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1) ;
    
    LL_DMA_InitTypeDef dma_rx_init = {0} ; 
    dma_rx_init.Channel = HW_DMA_IIC2_RX_CHANNEL ; 
    dma_rx_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY ;
    dma_rx_init.MemBurst = LL_DMA_MBURST_SINGLE ;
    dma_rx_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE ;
    dma_rx_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT ;
    dma_rx_init.Mode = LL_DMA_MODE_NORMAL ;                       
    dma_rx_init.PeriphBurst = LL_DMA_PBURST_SINGLE ;
    dma_rx_init.PeriphOrM2MSrcAddress = (uint32_t)&(HW_I2C_BMP390L_INSTANCE->DR) ;
    dma_rx_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE ;
    dma_rx_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT ; // 外设地址不递增
    dma_rx_init.Priority = LL_DMA_PRIORITY_HIGH ;
    
    dma_rx_init.MemoryOrM2MDstAddress = 0 ; 
    dma_rx_init.NbData = 0 ; 
    
    LL_DMA_Init(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM, &dma_rx_init) ;

    // 开启 RX 传输完成中断 (TC)
    LL_DMA_EnableIT_TC(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM) ;
}

void DMA_IIC2_Rx_Trigger(uint8_t *rx_buf, uint16_t len)
{
    // 失能 DMA 流，准备写入新参数
    LL_DMA_DisableStream(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM) ;

    // 动态注入目标地址和长度 
    LL_DMA_SetMemoryAddress(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM, (uint32_t)rx_buf) ;
    LL_DMA_SetDataLength(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM, len) ;

    // 清除 DMA 标志位 
    LL_DMA_ClearFlag_TC2(HW_DMA_IIC2_RX_INSTANCE) ;
    LL_DMA_ClearFlag_HT2(HW_DMA_IIC2_RX_INSTANCE) ;
    LL_DMA_ClearFlag_TE2(HW_DMA_IIC2_RX_INSTANCE) ;

    // I2C 专属设置：开启 Last DMA transfer 
    // 让硬件在最后一次 DMA 传输时自动在总线上发送 NACK，通知从机停止发送
    LL_I2C_EnableLastDMA(HW_I2C_BMP390L_INSTANCE) ;

    // 使能 I2C 外设的 DMA RX 请求
    LL_I2C_EnableDMAReq_RX(HW_I2C_BMP390L_INSTANCE) ;

    // 开启 DMA 接收流
    LL_DMA_EnableStream(HW_DMA_IIC2_RX_INSTANCE, HW_DMA_IIC2_RX_STREAM) ;
}

void DMA1_Stream2_IRQHandler(void) 
{
    if(LL_DMA_IsActiveFlag_TC2(HW_DMA_IIC2_RX_INSTANCE)) {
        LL_DMA_ClearFlag_TC2(HW_DMA_IIC2_RX_INSTANCE) ;

        if (s_dma_iic_tc_callbacks[DEVICE_IIC2] != NULL) 
        {
            s_dma_iic_tc_callbacks[DEVICE_IIC2]() ;
        }
    }
}