#include "tim/tim1.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

void tim1_dshot_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    LL_TIM_InitTypeDef TIM1_InitStructure = {0};
    TIM1_InitStructure.Prescaler = 0; 
    TIM1_InitStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM1_InitStructure.Autoreload = HW_TIM_DSHOT_ARR;
    TIM1_InitStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(HW_TIM_DSHOT_INSTANCE, &TIM1_InitStructure);
    
    // 配置 4 个 PWM 通道
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    
    // 循环初始化 CH1, CH2, CH3, CH4
    uint32_t channels[4] = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4};
    for(int i=0; i<4; i++) 
    {
        LL_TIM_OC_Init(HW_TIM_DSHOT_INSTANCE, channels[i], &TIM_OC_InitStruct);
        LL_TIM_OC_EnablePreload(HW_TIM_DSHOT_INSTANCE, channels[i]);
    }
    
    LL_TIM_EnableAllOutputs(HW_TIM_DSHOT_INSTANCE); // MOE = 1
    
    // 配置 Timer Burst 模式
    // Base Address = CCR1 (从 CCR1 开始写)
    // Length = 4 Transfers (写 CCR1, CCR2, CCR3, CCR4)
    LL_TIM_ConfigDMABurst(HW_TIM_DSHOT_INSTANCE, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
    
    // 启用 Update 事件的 DMA 请求
    LL_TIM_EnableDMAReq_UPDATE(HW_TIM_DSHOT_INSTANCE);
}
