#ifndef __CRSF_PARSER_H
#define __CRSF_PARSER_H

#include "CRSF_Protocol.h"
#include "stdint.h"

typedef struct {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_LQ;
    int8_t  uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_LQ;
    int8_t  downlink_SNR;
} crsf_link_statistics_t;

typedef struct {
    uint16_t ch0 ; // Roll
    uint16_t ch1 ; // Pitch
    uint16_t ch2 ; // Throttle
    uint16_t ch3 ; // Yaw
    uint16_t ch4 ; // AUX 1 (Arm 锁桨开关)
    //以下数据功能可以自由定义
    uint16_t ch5 ;
    uint16_t ch6 ;
    uint16_t ch7 ;
    uint16_t ch8 ;
    uint16_t ch9 ;
    uint16_t ch10 ;
    uint16_t ch11 ;
    uint16_t ch12 ;
    uint16_t ch13 ;
    uint16_t ch14 ;
    uint16_t ch15 ;
} crsf_channels_t;

uint16_t CRSF_Channel_To_PWM(uint16_t raw_value)  ; 
void CRSF_Parser_Byte(uint8_t byte) ; 

/** 任务上下文读取（关中断拷贝，禁止自旋） */
void CRSF_Get_Channels(crsf_channels_t* out_data) ;

/** 控制环/ISR 读取双缓冲快照（无锁、无自旋） */
void CRSF_ReadChannels_Control(crsf_channels_t* out_data) ;

#endif
