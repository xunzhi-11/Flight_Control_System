#include "adc/ll_adc.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"

void ADC_FOR_Joystick_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1); 

    LL_ADC_InitTypeDef adc_for_joystick_initstructure = {0}; 

    adc_for_joystick_initstructure.Resolution = LL_ADC_RESOLUTION_12B;
    adc_for_joystick_initstructure.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
    adc_for_joystick_initstructure.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT; 

    LL_ADC_Init(ADC1 , &adc_for_joystick_initstructure); 

    LL_ADC_REG_InitTypeDef adc_reg_init = {0};
    
    // 绑定触发源为 TIM4_CH4 OC
    adc_reg_init.TriggerSource    = HW_ADC_JOY_TRIGGER_SOURCE; 
    
    // 关闭连续转换
    adc_reg_init.ContinuousMode   = LL_ADC_REG_CONV_SINGLE; 
    
    // 开启 DMA 请求
    // 告诉 ADC：每次转换完，都自动给 DMA 发信号请求搬运
    adc_reg_init.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; 
    
    adc_reg_init.SequencerLength  = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS; // 序列长度设为 4 个通道
    adc_reg_init.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;    // 禁用间断模式
    
    LL_ADC_REG_Init(HW_ADC_JOY_INSTANCE, &adc_reg_init);

    // 设置外部触发的极性 (上升沿触发)
    LL_ADC_REG_StartConversionExtTrig(HW_ADC_JOY_INSTANCE, LL_ADC_REG_TRIG_EXT_RISING);

    // RANK1~4: PA1 Throttle, PA2 Yaw, PA3 Pitch, PA4 Roll
    LL_ADC_REG_SetSequencerRanks(HW_ADC_JOY_INSTANCE, LL_ADC_REG_RANK_1, HW_ADC_JOY_CH1);
    LL_ADC_SetChannelSamplingTime(HW_ADC_JOY_INSTANCE, HW_ADC_JOY_CH1, LL_ADC_SAMPLINGTIME_84CYCLES);
    
    LL_ADC_REG_SetSequencerRanks(HW_ADC_JOY_INSTANCE, LL_ADC_REG_RANK_2, HW_ADC_JOY_CH2);
    LL_ADC_SetChannelSamplingTime(HW_ADC_JOY_INSTANCE, HW_ADC_JOY_CH2, LL_ADC_SAMPLINGTIME_84CYCLES);
    
    LL_ADC_REG_SetSequencerRanks(HW_ADC_JOY_INSTANCE, LL_ADC_REG_RANK_3, HW_ADC_JOY_CH3);
    LL_ADC_SetChannelSamplingTime(HW_ADC_JOY_INSTANCE, HW_ADC_JOY_CH3, LL_ADC_SAMPLINGTIME_84CYCLES);
    
    LL_ADC_REG_SetSequencerRanks(HW_ADC_JOY_INSTANCE, LL_ADC_REG_RANK_4, HW_ADC_JOY_CH4);
    LL_ADC_SetChannelSamplingTime(HW_ADC_JOY_INSTANCE, HW_ADC_JOY_CH4, LL_ADC_SAMPLINGTIME_84CYCLES);

    LL_ADC_Enable(HW_ADC_JOY_INSTANCE);
}