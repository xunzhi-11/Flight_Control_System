#include "dma/ll_dma_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"
#include "sys_hardware_config.h"

static void (*s_dma_spi_tc_callbacks[3])(void) = {NULL} ; 

// 注册接口
void DMA_SPI_RegisterTCCallback(uint8_t spi_id, void (*callback)(void)) 
{
    s_dma_spi_tc_callbacks[spi_id] = callback ;
}

void dma_to_spi1_rx_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2) ;
    
    LL_DMA_InitTypeDef dma_rx_init = {0} ; 
    dma_rx_init.Channel = HW_DMA_SPI1_RX_CHANNEL ; 
    dma_rx_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY ;
    dma_rx_init.MemBurst = LL_DMA_MBURST_SINGLE ;
    dma_rx_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE ;
    dma_rx_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT ;
    dma_rx_init.Mode = LL_DMA_MODE_NORMAL ;                       
    dma_rx_init.PeriphBurst = LL_DMA_PBURST_SINGLE ;
    dma_rx_init.PeriphOrM2MSrcAddress = (uint32_t)&(SPI1->DR) ;   // 源就是 SPI1 的 DR 寄存器
    dma_rx_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE ;
    dma_rx_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT ; // 外设地址不递增
    dma_rx_init.Priority = LL_DMA_PRIORITY_HIGH ;
    
    dma_rx_init.MemoryOrM2MDstAddress = 0 ; 
    dma_rx_init.NbData = 0 ; 
    
    LL_DMA_Init(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM, &dma_rx_init) ;

    LL_DMA_InitTypeDef dma_tx_init = dma_rx_init ; 
    dma_tx_init.Channel = HW_DMA_SPI1_TX_CHANNEL ;
    dma_tx_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH ;
    dma_tx_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT ; 
    dma_tx_init.PeriphOrM2MSrcAddress = (uint32_t)&(SPI1->DR) ;
    
    dma_tx_init.MemoryOrM2MDstAddress = 0 ; 
    dma_tx_init.NbData = 0 ; 
    
    LL_DMA_Init(HW_DMA_SPI1_TX_INSTANCE, HW_DMA_SPI1_TX_STREAM, &dma_tx_init) ;

    // 开启 RX 传输完成中断 (TC)
    LL_DMA_EnableIT_TC(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM) ;
}

void DMA_SPI1_TxRx_Trigger(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    // 失能 DMA 流，准备写入新参数
    LL_DMA_DisableStream(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM) ;
    LL_DMA_DisableStream(HW_DMA_SPI1_TX_INSTANCE, HW_DMA_SPI1_TX_STREAM) ;

    // 动态注入目标地址和长度 
    LL_DMA_SetMemoryAddress(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM, (uint32_t)rx_buf) ;
    LL_DMA_SetMemoryAddress(HW_DMA_SPI1_TX_INSTANCE, HW_DMA_SPI1_TX_STREAM, (uint32_t)tx_buf) ;
    LL_DMA_SetDataLength(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM, len) ;
    LL_DMA_SetDataLength(HW_DMA_SPI1_TX_INSTANCE, HW_DMA_SPI1_TX_STREAM, len) ;

    // 清除 DMA 标志位 
    LL_DMA_ClearFlag_TC0(HW_DMA_SPI1_RX_INSTANCE) ;
    LL_DMA_ClearFlag_HT0(HW_DMA_SPI1_RX_INSTANCE) ;
    LL_DMA_ClearFlag_TE0(HW_DMA_SPI1_RX_INSTANCE) ;
    
    LL_DMA_ClearFlag_TC3(HW_DMA_SPI1_TX_INSTANCE) ;
    LL_DMA_ClearFlag_HT3(HW_DMA_SPI1_TX_INSTANCE) ;
    LL_DMA_ClearFlag_TE3(HW_DMA_SPI1_TX_INSTANCE) ;

    // 先开 RX，后开 TX 
    LL_DMA_EnableStream(HW_DMA_SPI1_RX_INSTANCE, HW_DMA_SPI1_RX_STREAM) ;
    LL_DMA_EnableStream(HW_DMA_SPI1_TX_INSTANCE, HW_DMA_SPI1_TX_STREAM) ;

    // 使能 SPI 外设的 DMA 请求
    LL_SPI_EnableDMAReq_RX(SPI1) ;
    LL_SPI_EnableDMAReq_TX(SPI1) ;
}

void DMA2_Stream0_IRQHandler(void) 
{
    if(LL_DMA_IsActiveFlag_TC0(HW_DMA_SPI1_RX_INSTANCE)) 
    {
        LL_DMA_ClearFlag_TC0(HW_DMA_SPI1_RX_INSTANCE) ;
        if (s_dma_spi_tc_callbacks[DEVICE_SPI1] != NULL) 
        {
            s_dma_spi_tc_callbacks[DEVICE_SPI1]() ;
        }
    }
}

void dma_to_spi2_rx_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1) ;
    
    LL_DMA_InitTypeDef dma_rx_init = {0} ; 
    dma_rx_init.Channel = HW_DMA_SPI2_RX_CHANNEL ; 
    dma_rx_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY ;
    dma_rx_init.MemBurst = LL_DMA_MBURST_SINGLE ;
    dma_rx_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE ;
    dma_rx_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT ;
    dma_rx_init.Mode = LL_DMA_MODE_NORMAL ;                       
    dma_rx_init.PeriphBurst = LL_DMA_PBURST_SINGLE ;
    dma_rx_init.PeriphOrM2MSrcAddress = (uint32_t)&(SPI2->DR) ;   
    dma_rx_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE ;
    dma_rx_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT ; 
    dma_rx_init.Priority = LL_DMA_PRIORITY_HIGH ;
    
    dma_rx_init.MemoryOrM2MDstAddress = 0 ; 
    dma_rx_init.NbData = 0 ; 
    
    LL_DMA_Init(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM, &dma_rx_init) ;

    LL_DMA_InitTypeDef dma_tx_init = dma_rx_init ; 
    dma_tx_init.Channel = HW_DMA_SPI2_TX_CHANNEL ;
    dma_tx_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH ;
    dma_tx_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT ; 
    dma_tx_init.PeriphOrM2MSrcAddress = (uint32_t)&(SPI2->DR) ;
    
    dma_tx_init.MemoryOrM2MDstAddress = 0 ; 
    dma_tx_init.NbData = 0 ; 
    
    LL_DMA_Init(HW_DMA_SPI2_TX_INSTANCE, HW_DMA_SPI2_TX_STREAM, &dma_tx_init) ;

    // 开启 RX 传输完成中断 (TC)
    LL_DMA_EnableIT_TC(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM) ;
}

void DMA_SPI2_TxRx_Trigger(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    // 失能 DMA 流，准备写入新参数
    LL_DMA_DisableStream(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM) ;
    LL_DMA_DisableStream(HW_DMA_SPI2_TX_INSTANCE, HW_DMA_SPI2_TX_STREAM) ;

    // 动态注入目标地址和长度 
    LL_DMA_SetMemoryAddress(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM, (uint32_t)rx_buf) ;
    LL_DMA_SetMemoryAddress(HW_DMA_SPI2_TX_INSTANCE , HW_DMA_SPI2_TX_STREAM, (uint32_t)tx_buf) ; 
    LL_DMA_SetDataLength(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM, len) ;
    LL_DMA_SetDataLength(HW_DMA_SPI2_TX_INSTANCE, HW_DMA_SPI2_TX_STREAM, len) ;

    // 清除 DMA 标志位 (Stream3 对应标志位包含 3，Stream4 对应标志位包含 4)
    LL_DMA_ClearFlag_TC3(HW_DMA_SPI2_RX_INSTANCE) ;
    LL_DMA_ClearFlag_HT3(HW_DMA_SPI2_RX_INSTANCE) ;
    LL_DMA_ClearFlag_TE3(HW_DMA_SPI2_RX_INSTANCE) ;
    
    LL_DMA_ClearFlag_TC4(HW_DMA_SPI2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_HT4(HW_DMA_SPI2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_TE4(HW_DMA_SPI2_TX_INSTANCE) ;

    // 先开 RX，后开 TX 
    LL_DMA_EnableStream(HW_DMA_SPI2_RX_INSTANCE, HW_DMA_SPI2_RX_STREAM) ;
    LL_DMA_EnableStream(HW_DMA_SPI2_TX_INSTANCE, HW_DMA_SPI2_TX_STREAM) ;

    // 使能 SPI 外设的 DMA 请求
    LL_SPI_EnableDMAReq_RX(SPI2) ;
    LL_SPI_EnableDMAReq_TX(SPI2) ;
}

void DMA1_Stream3_IRQHandler(void) 
{
    if(LL_DMA_IsActiveFlag_TC3(HW_DMA_SPI2_RX_INSTANCE)) {
        LL_DMA_ClearFlag_TC3(HW_DMA_SPI2_RX_INSTANCE) ;

        if (s_dma_spi_tc_callbacks[DEVICE_SPI2] != NULL) 
        {
            s_dma_spi_tc_callbacks[DEVICE_SPI2]() ;
        }
    }
}