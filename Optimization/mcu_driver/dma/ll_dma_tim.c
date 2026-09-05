#include "dma/ll_dma_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "sys_hardware_config.h"
#include "dshot_hw/ll_dshot_driver.h"
#include "utils/dwt/dwt_profiler.h"

void dma_to_tim1_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    
    LL_DMA_InitTypeDef DMA2_InitStructure = {0};
    DMA2_InitStructure.Channel = HW_DMA_TIM1_CHANNEL; 
    DMA2_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA2_InitStructure.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
    DMA2_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
    DMA2_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
    DMA2_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
    
    DMA2_InitStructure.MemoryOrM2MDstAddress = (uint32_t)HW_DShot_Get_Buffer_Addr();
    
    //  DMA 目标地址改为 TIM1 的 DMAR (Burst Register)
    // 这是一个虚拟寄存器，写入它会根据 Burst 设置分发给 CCR1-CCR4
    DMA2_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM1->DMAR; 
    
    DMA2_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    DMA2_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    DMA2_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    
    //  外设地址不自增 (始终写入 DMAR)
    DMA2_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT; 
    
    DMA2_InitStructure.Mode = LL_DMA_MODE_NORMAL;
    DMA2_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
    
    // 长度在 KickOff 时设置，这里先给 0
    DMA2_InitStructure.NbData = 0; 
    
    LL_DMA_Init(HW_DMA_TIM1_INSTANCE, HW_DMA_TIM1_STREAM, &DMA2_InitStructure); 

    LL_DMA_EnableIT_TC(HW_DMA_TIM1_INSTANCE, HW_DMA_TIM1_STREAM);
}


void DMA2_Stream5_IRQHandler(void)
{
   if(LL_DMA_IsActiveFlag_TC5(HW_DMA_TIM1_INSTANCE))
   {
       LL_DMA_ClearFlag_TC5(HW_DMA_TIM1_INSTANCE);
       LL_TIM_DisableCounter(HW_TIM_DSHOT_INSTANCE);
   }
}