#ifndef __SYSTEM_CONFIG_TYPES_H
#define __SYSTEM_CONFIG_TYPES_H

#include "stdio.h"


#define SYS_BATT_ALARM_DEFAULT  35
#define SYS_BACK_LIGHT_DEFAULT  60

typedef enum {
    ELRS_PARAM_PACKET_RATE   = 1 ,
    ELRS_PARAM_TELEM_RATIO   = 2 ,
    ELRS_PARAM_SWITCH_MODE   = 3 ,
    ELRS_PARAM_MODEL_MATCH   = 4 ,
    ELRS_PARAM_MAX_POWER     = 6 ,
    ELRS_PARAM_DYNAMIC_POWER = 7 ,
    ELRS_PARAM_BAND          = 9 ,
    ELRS_PARAM_CHANNEL       = 10 , 
    ELRS_PARAM_PWR_LVL       = 11 , 
    // 命令类 ID
    ELRS_CMD_SEND_VTX        = 13 ,
    ELRS_CMD_ENABLE_WIFI     = 15 ,
    ELRS_CMD_BIND            = 18
} ELRS_Param_ID_e ;

// 每个特定的参数，定义它的值域枚举 (强类型约束，防写错)
typedef enum {
    PKT_RATE_50HZ      = 0 ,
    PKT_RATE_100HZ     = 1 ,
    PKT_RATE_150HZ     = 2 ,
    PKT_RATE_250HZ     = 3 ,
    PKT_RATE_333HZ     = 4
} ELRS_PacketRate_e ;

typedef enum {
    PWR_100MW      = 0 ,
    PWR_10MW       = 1 ,
} ELRS_MaxPower_e ;

typedef enum
{
    TELEM_RATIO_DEFAULT = 0 , 
    TELEM_RATIO_OFF  = 1 , 
    TELEM_RATIO_1_128  = 2 ,
    TELEM_RATIO_1_64  = 3 ,  
    TELEM_RATIO_1_32  = 4 ,  
    TELEM_RATIO_1_16  = 5 ,  
    TELEM_RATIO_1_8  = 6 ,  
    TELEM_RATIO_1_4  = 7 ,  
    TELEM_RATIO_1_2  = 8 ,  
    TELEM_RATIO_1_1  = 9   

} ELRS_TELEM_RATIO_e ; 

typedef enum 
{
    SEND_MODE_8CH            = 0 , 
    SEND_MODE_16CH_HALF_RATE = 1 , 
    SEND_MODE_12CH_MIXED     = 2
} ELRS_SEND_MODE_e ;

typedef enum
{
    MAX_POWER_10_MW  = 0 , 
    MAX_POWER_25_MW  = 1 , 
    MAX_POWER_50_MW  = 2 , 
    MAX_POWER_100_MW = 3 , 
    MAX_POWER_250_MW = 4 , 
    MAX_POWER_500_MW = 5 , 

} ELRS_MAX_POWER_e ; 

typedef enum
{
    BAND_OFF = 0 , 
    BAND_A   = 1 , 
    BAND_B   = 2 , 
    BAND_E   = 3 , 
    BAND_L   = 4 , 
    BAND_F   = 5  
} ELRS_BAND_e ; 

typedef enum
{
    CHANNEL_1 = 0 , 
    CHANNEL_2 = 1 , 
    CHANNEL_3 = 2 , 
    CHANNEL_4 = 3 , 
    CHANNEL_5 = 4 , 
    CHANNEL_6 = 5 , 
    CHANNEL_7 = 6 , 
    CHANNEL_8 = 7 
} ELRS_CHANNEL_e ; 

typedef enum
{
    PWR_LVL_DEFAULT = 0 , 
    PWR_LVL_0       = 1 , 
    PWR_LVL_1       = 2 , 
    PWR_LVL_2       = 3 , 
    PWR_LVL_3       = 4 , 
    PWR_LVL_4       = 5 , 
    PWR_LVL_5       = 6 , 
    PWR_LVL_6       = 7 , 
    PWR_LVL_7       = 8 
} ELRS_PWR_LVL_e ; 

typedef enum
{
    NOT_BIND = 0 , 
    IS_BIND = 1
} ELRS_Bind_State_e ; 




#endif
