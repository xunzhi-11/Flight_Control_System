#ifndef __CRSF_PARSER_H
#define __CRSF_PARSER_H

#include "CRSF_Protocol.h"
#include "stdint.h"


// 电池遥测结构体
typedef struct {
    uint16_t voltage_01v ;   // 电压 (单位: 0.1V)
    uint16_t current_01a ;   // 电流 (单位: 0.1A)
    uint32_t capacity_mah ;  // 消耗电量 (单位: mAh)
    uint8_t  remaining_pct ; // 剩余电量百分比 (0~100)
} crsf_telemetry_battery_t ;

// GPS 遥测结构体
typedef struct {
    int32_t  latitude ;      // 纬度 (度 * 10^7)
    int32_t  longitude ;     // 经度 (度 * 10^7)
    uint16_t ground_speed ;  // 地速 (km/h * 10)
    uint16_t heading ;       // 航向 (度 * 100)
    uint16_t alttitude_m ;    // 真实海拔高度 (单位: m)
    uint8_t  satellites ;    // 搜星数量
} crsf_telemetry_gps_t ;

// 姿态遥测结构体
typedef struct {
    int16_t pitch_rad_10k ;  // 俯仰角 (弧度 * 10000)
    int16_t roll_rad_10k ;   // 横滚角 (弧度 * 10000)
    int16_t yaw_rad_10k ;    // 偏航角 (弧度 * 10000)
} crsf_telemetry_attitude_t ;

//链路状态结构体
typedef struct {
    uint8_t uplink_RSSI_1 ;
    uint8_t uplink_RSSI_2 ;
    uint8_t uplink_LQ ;
    int8_t  uplink_SNR ;
    uint8_t active_antenna ;
    uint8_t rf_Mode ;
    uint8_t uplink_TX_Power ;
    uint8_t downlink_RSSI ;
    uint8_t downlink_LQ ;
    int8_t  downlink_SNR ;
} crsf_link_statistics_t ;

// 设备信息结构体
typedef struct {
    char     device_name[32] ;
    uint32_t serial_number ;
    uint32_t hardware_version ;
    uint8_t  software_version[4] ; 
    uint8_t  parameter_count ;
    uint8_t  parameter_version ;
} crsf_device_info_t ;

void CRSF_Parser_Byte(uint8_t byte) ; 
uint8_t CRSF_Get_Parameter_Count(void) ;
uint8_t CRSF_Get_Last_Received_Param_ID(void) ;
crsf_device_info_t* CRSF_Get_Device_Info_Ptr(void) ; 
uint8_t CRSF_Get_Last_Param_Data(uint8_t* buffer_out) ; 
crsf_link_statistics_t* CRSF_Get_Link_Stats_Ptr(void) ; 
crsf_telemetry_battery_t* CRSF_Get_Battery_Ptr(void) ;
crsf_telemetry_gps_t* CRSF_Get_GPS_Ptr(void) ;
uint32_t CRSF_Get_Link_Stats_Seq(void) ;
uint32_t CRSF_Get_Battery_Seq(void) ;
uint32_t CRSF_Get_GPS_Seq(void) ;
uint32_t CRSF_Get_Module_Rx_Seq(void) ;

#endif
