#ifndef __TIME5_H
#define __TIME5_H

#include "stdint.h"

void Time5_Stamp_Init(void) ; 
uint32_t TIM5_GetStamp(void) ;
/** 两次函数调用之间的间隔 (s)，*last_tick 会被更新为当前 TIM5->CNT */
float TIM5_GetDt(uint32_t *last_tick) ;
/** 两次传感器事件时间戳之间的间隔 (s)，*last_event_stamp 更新为 now_stamp；首帧返回 0 */
float TIM5_DtFromEventStamp(uint32_t now_stamp, uint32_t *last_event_stamp) ;

#endif
