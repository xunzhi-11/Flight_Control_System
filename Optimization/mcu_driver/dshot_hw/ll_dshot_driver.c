#include "ll_dshot_driver.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"

static uint16_t dshot_dma_buffer[24 * HW_DSHOT_MOTOR_NUM]; 

uint16_t* HW_DShot_Get_Buffer_Addr(void) 
{
    return dshot_dma_buffer;
}

void dshot_trigger(uint16_t m1_pkt, uint16_t m2_pkt, uint16_t m3_pkt, uint16_t m4_pkt)
{
    for (int i = 0; i < 16; i++) 
    {
        uint16_t mask = 0x8000 >> i;
        dshot_dma_buffer[i * 4 + 0] = (m1_pkt & mask) ? HW_TIM_DSHOT_BIT_1 : HW_TIM_DSHOT_BIT_0;
        dshot_dma_buffer[i * 4 + 1] = (m2_pkt & mask) ? HW_TIM_DSHOT_BIT_1 : HW_TIM_DSHOT_BIT_0;
        dshot_dma_buffer[i * 4 + 2] = (m3_pkt & mask) ? HW_TIM_DSHOT_BIT_1 : HW_TIM_DSHOT_BIT_0;
        dshot_dma_buffer[i * 4 + 3] = (m4_pkt & mask) ? HW_TIM_DSHOT_BIT_1 : HW_TIM_DSHOT_BIT_0;
    }

    // 填充尾部的 0 (帧间隔)
    for (int i = 16 * 4; i < 24 * 4; i++) {
        dshot_dma_buffer[i] = 0;
    }

    LL_TIM_DisableCounter(HW_TIM_DSHOT_INSTANCE);
    LL_DMA_DisableStream(HW_DMA_TIM1_INSTANCE, HW_DMA_TIM1_STREAM);
    
    LL_DMA_ClearFlag_TC5(HW_DMA_TIM1_INSTANCE);
    LL_DMA_ClearFlag_HT5(HW_DMA_TIM1_INSTANCE);
    LL_TIM_SetCounter(HW_TIM_DSHOT_INSTANCE, 0);
    
    LL_DMA_SetDataLength(HW_DMA_TIM1_INSTANCE, HW_DMA_TIM1_STREAM, 24 * HW_DSHOT_MOTOR_NUM);
    LL_DMA_EnableStream(HW_DMA_TIM1_INSTANCE, HW_DMA_TIM1_STREAM);
    
    LL_TIM_GenerateEvent_UPDATE(HW_TIM_DSHOT_INSTANCE);
    LL_TIM_ClearFlag_UPDATE(HW_TIM_DSHOT_INSTANCE);
    LL_TIM_EnableCounter(HW_TIM_DSHOT_INSTANCE);
}