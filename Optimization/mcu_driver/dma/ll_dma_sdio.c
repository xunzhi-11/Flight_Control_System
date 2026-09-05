#include "dma/ll_dma_sdio.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"
#include "sys_software_config.h"

static volatile SDIO_DMA_State_e sdio_dma_state = SDIO_DMA_IDLE;

void dma_to_sdio_init(void)
{
    // 使能DMA2时钟
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    // 先禁用Stream
    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM);
    while (LL_DMA_IsEnabledStream(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM));

    // 清除所有中断标志
    LL_DMA_ClearFlag_TC6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_HT6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_TE6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_DME6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_FE6(HW_DMA_SDIO_INSTANCE);

    // 配置DMA（基本配置，方向在传输时设置）
    LL_DMA_SetChannelSelection(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, HW_DMA_SDIO_CHANNEL);
    LL_DMA_SetDataLength(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, 0);
    LL_DMA_SetPeriphAddress(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, (uint32_t)&SDIO->FIFO);

    // 数据宽度：32位
    LL_DMA_SetPeriphSize(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_PDATAALIGN_WORD);
    LL_DMA_SetMemorySize(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_MDATAALIGN_WORD);

    // 地址增量：内存增加，外设不增加
    LL_DMA_SetMemoryIncMode(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphIncMode(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_PERIPH_NOINCREMENT);

    // 模式：普通模式（非循环）
    LL_DMA_SetMode(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_MODE_NORMAL);

    // 优先级：非常高（SD卡数据传输关键）
    LL_DMA_SetStreamPriorityLevel(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_PRIORITY_VERYHIGH);


    // FIFO配置 - SDIO需要4字突发
    LL_DMA_EnableFifoMode(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM);
    LL_DMA_SetFIFOThreshold(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_FIFOTHRESHOLD_FULL);

    // Burst: SDIO 写路径用 SINGLE，INC4 易触发 DCRCFAIL
    LL_DMA_SetMemoryBurstxfer(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_MBURST_SINGLE);
    LL_DMA_SetPeriphBurstxfer(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM, LL_DMA_PBURST_SINGLE);

    // 外设流控制 - SDIO控制传输长度
    DMA2_Stream6->CR |= DMA_SxCR_PFCTRL;

    // 使能中断
    LL_DMA_EnableIT_TC(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM);   // 传输完成中断
    LL_DMA_EnableIT_TE(HW_DMA_SDIO_INSTANCE,  HW_DMA_SDIO_STREAM);   // 传输错误中断

    sdio_dma_state = SDIO_DMA_IDLE;
}

void dma_sdio_SetState(SDIO_DMA_State_e state)
{
    sdio_dma_state = state;
}

SDIO_DMA_State_e dma_sdio_GetState(void)
{
    return sdio_dma_state;
}

void dma_sdio_ClearState(void)
{
    sdio_dma_state = SDIO_DMA_IDLE;
}

void DMA2_Stream6_IRQHandler(void)
{
    // 传输完成
    if (LL_DMA_IsActiveFlag_TC6(HW_DMA_SDIO_INSTANCE))
    {
        LL_DMA_ClearFlag_TC6(HW_DMA_SDIO_INSTANCE);
        sdio_dma_state = SDIO_DMA_COMPLETE;
    }

    // 传输错误
    if (LL_DMA_IsActiveFlag_TE6(HW_DMA_SDIO_INSTANCE))
    {
        LL_DMA_ClearFlag_TE6(HW_DMA_SDIO_INSTANCE);
        sdio_dma_state = SDIO_DMA_ERROR;
    }

    // FIFO错误（通常可忽略）
    if (LL_DMA_IsActiveFlag_FE6(HW_DMA_SDIO_INSTANCE))
    {
        LL_DMA_ClearFlag_FE6(HW_DMA_SDIO_INSTANCE);
    }

    // 直接模式错误
    if (LL_DMA_IsActiveFlag_DME6(HW_DMA_SDIO_INSTANCE))
    {
        LL_DMA_ClearFlag_DME6(HW_DMA_SDIO_INSTANCE);
        sdio_dma_state = SDIO_DMA_ERROR;
    }
}
