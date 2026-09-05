#include "stm32f4xx.h"                  // Device header
#include "tim5.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"

void Time5_Stamp_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5) ; 

    LL_TIM_InitTypeDef Time_Stamp_InitStructure = {0} ; 

    Time_Stamp_InitStructure.Autoreload = 0xFFFFFFFF;//32位寄存器最大值
    Time_Stamp_InitStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    Time_Stamp_InitStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
    Time_Stamp_InitStructure.Prescaler = 84 - 1;

    LL_TIM_Init(TIM5 , &Time_Stamp_InitStructure) ; 

    LL_TIM_DisableARRPreload(TIM5);

    // 开启计数器
    LL_TIM_EnableCounter(TIM5);
    
    // 产生一次更新事件以加载预分频器值 (Good Practice)
    LL_TIM_GenerateEvent_UPDATE(TIM5);
}


// 获取当前微秒时间戳 (32位，约71分钟溢出一次)
__attribute__((always_inline)) inline uint32_t TIM5_GetStamp(void) 
{
    return TIM5->CNT;
}


// 适用于计算 dt(s)
inline float TIM5_GetDt(uint32_t *last_tick) 
{
    uint32_t now = TIM5_GetStamp();
    uint32_t diff;
    
    // 无符号减法自动处理溢出 (间隔不超过71分钟)
    if (now >= *last_tick) {
        diff = now - *last_tick;
    } else {
        diff = (0xFFFFFFFF - *last_tick) + 1 + now;
    }
    
    *last_tick = now;
    return (float)diff * 0.000001f; // 转换为秒
}

float TIM5_DtFromEventStamp(uint32_t now_stamp, uint32_t *last_event_stamp)
{
    if (last_event_stamp == NULL) {
        return 0.0f;
    }

    if (*last_event_stamp == 0u) {
        *last_event_stamp = now_stamp;
        return 0.0f;
    }

    if (now_stamp == *last_event_stamp) {
        return 0.0f;
    }

    uint32_t diff;
    if (now_stamp >= *last_event_stamp) {
        diff = now_stamp - *last_event_stamp;
    } else {
        diff = (0xFFFFFFFFu - *last_event_stamp) + 1u + now_stamp;
    }

    *last_event_stamp = now_stamp;
    return (float)diff * 0.000001f;
}
