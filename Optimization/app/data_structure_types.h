#ifndef __DATA_STRUCTURE_TYPES_H
#define __DATA_STRUCTURE_TYPES_H

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

// 纯原始数据（16位，未缩放）
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t irq_timestamp;
} IMU_Raw_t;

// 物理量数据（浮点，已缩放为 g 和 dps/rad）
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    uint32_t timestamp;
    float dt ; 
} IMU_Scaled_t;

typedef struct 
{
    uint32_t press; 
    uint32_t temp;  
    uint32_t god_time ; 
}BAROMETER_Raw_t;

typedef struct 
{
    uint32_t god_time ; 
    float dt ; 
    double press ; 
    double temp ;
}BAROMETER_Real_t;

typedef struct 
{
    float mag_x , mag_y , mag_z ; 
    float Total_Filed_Intensity ; 
    float dt ; 
    int32_t god_time ; 
}MAGNETOMETER_Calib_t ; 

typedef struct 
{
    float mag_x ; 
    float mag_y ; 
    float mag_z ; 
    uint32_t god_time ;   
} MAGNETOMETER_Trimed_t ;


// 电池遥测结构体
typedef struct {
    uint16_t voltage_01v;   // 电压 (单位: 0.1V)
    uint16_t current_01a;   // 电流 (单位: 0.1A)
    uint32_t capacity_mah;  // 消耗电量 (单位: mAh)
    uint8_t  remaining_pct; // 剩余电量百分比 (0~100)
} crsf_telemetry_battery_t;

// GPS 遥测结构体
typedef struct {
    int32_t  latitude;      // 纬度 (度 * 10^7)
    int32_t  longitude;     // 经度 (度 * 10^7)
    uint16_t ground_speed;  // 地速 (km/h * 10)
    uint16_t heading;       // 航向 (度 * 100)
    uint16_t altitude_m;    // 真实海拔高度 (单位: m)
    uint8_t  satellites;    // 搜星数量
} crsf_telemetry_gps_t;

// 姿态遥测结构体
typedef struct {
    int16_t pitch_rad_10k;  // 俯仰角 (弧度 * 10000)
    int16_t roll_rad_10k;   // 横滚角 (弧度 * 10000)
    int16_t yaw_rad_10k;    // 偏航角 (弧度 * 10000)
} crsf_telemetry_attitude_t;

typedef struct {
    // 日期与时间 (UTC)
    uint16_t year;          // 实际年份
    uint8_t  month;         // 1 ~ 12
    uint8_t  day;           // 1 ~ 31
    uint8_t  hour;          // 0 ~ 23
    uint8_t  minute;        // 0 ~ 59
    uint8_t  second;        // 0 ~ 59
    uint16_t msec;          // 毫秒: 0 ~ 999

    // 位置信息
    double   latitude;      // 十进制纬度
    double   longitude;     // 十进制经度
    float    altitude;      // 海拔高度 (米)

    // 运动状态
    float    speed_kmh;     // 地面速度 (公里/小时)
    float    course;        // 地面航向角 (度)

    // 定位质量与状态
    bool     is_valid;      // true = 定位数据有效 ('A'), false = 数据无效 ('V')
    uint8_t  fix_quality;   // GGA定位指示: 0=未定位, 1=单点定位, 2=差分
    uint8_t  satellites;    // 参与定位的卫星数量
    float    hdop;          // 水平精度因子
} atgm336h_data_t;

#endif