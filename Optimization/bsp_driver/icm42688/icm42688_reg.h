#ifndef ICM42688_REG_H
#define ICM42688_REG_H


//常用寄存器地址


//BANK0
//数据寄存器
#define ICM42688_TEMP_DATA1                 0x1D
#define ICM42688_TEMP_DATA0                 0x1E
#define ICM42688_ACCEL_DATA_X1              0x1F
#define ICM42688_ACCEL_DATA_X0              0x20
#define ICM42688_ACCEL_DATA_Y1              0x21
#define ICM42688_ACCEL_DATA_Y0              0x22
#define ICM42688_ACCEL_DATA_Z1              0x23
#define ICM42688_ACCEL_DATA_Z0              0x24
#define ICM42688_GYRO_DATA_X1               0x25
#define ICM42688_GYRO_DATA_X0               0x26
#define ICM42688_GYRO_DATA_Y1               0x27
#define ICM42688_GYRO_DATA_Y0               0x28
#define ICM42688_GYRO_DATA_Z1               0x29
#define ICM42688_GYRO_DATA_Z0               0x2A
//传感器配置
#define ICM42688_PWR_MGMT0                  0x4E
#define ICM42688_GYRO_CONFIG0               0x4F
#define ICM42688_ACCEL_CONFIG0              0x50
#define ICM42688_GYRO_CONFIG1               0x51
#define ICM42688_GYRO_ACCEL_CONFIG0         0x52
#define ICM42688_ACCEL_CONFIG1              0x53
//芯片id地址
#define ICM42688_WHO_AM_I                    0x75
//切换分区
#define ICM42688_REG_BANK_SEL                0x76


//==================================================================================================//
// BANK选择宏
#define ICM42688_BANK0                      0x00
#define ICM42688_BANK1                      0x01
#define ICM42688_BANK2                      0x02
#define ICM42688_BANK3                      0x03
#define ICM42688_BANK4                      0x04
//芯片id
#define ICM42688_WHO_AM_I_VALUE             0x47

// PWR_MGMT0 配置
#define ICM42688_PWR_MGMT0_ACCEL_LN         (0x0F)  // 加速度计低噪声模式
#define ICM42688_PWR_MGMT0_GYRO_LN          (0xF0)  // 陀螺仪低噪声模式
#define ICM42688_PWR_MGMT0_LN                0xFF   //组合定义


// 电源管理
#define ICM42688_PWR_MGMT0_TEMP_DIS            (1 << 5)
#define ICM42688_PWR_MGMT0_IDLE                (1 << 4)
#define ICM42688_PWR_MGMT0_GYRO_MODE_MASK      0x0C
#define ICM42688_PWR_MGMT0_GYRO_MODE_SHIFT     2
#define ICM42688_PWR_MGMT0_ACCEL_MODE_MASK     0x03
#define ICM42688_PWR_MGMT0_ACCEL_MODE_SHIFT    0

// 模式选择
#define ICM42688_SENSOR_MODE_OFF               0x00  //关机
#define ICM42688_SENSOR_MODE_LOW_NOISE         0x01 //低噪声
#define ICM42688_SENSOR_MODE_LOW_POWER         0x02 //低功耗

// 加速度计量程
#define ICM42688_ACCEL_FS_16G                  0x01  // ±16g
#define ICM42688_ACCEL_FS_8G                   0x02  // ±8g
#define ICM42688_ACCEL_FS_4G                   0x03  // ±4g
#define ICM42688_ACCEL_FS_2G                   0x04  // ±2g

// 陀螺仪量程
#define ICM42688_GYRO_FS_2000DPS               0x00  // ±2000 dps
#define ICM42688_GYRO_FS_1000DPS               0x01  // ±1000 dps
#define ICM42688_GYRO_FS_500DPS                0x02  // ±500 dps
#define ICM42688_GYRO_FS_250DPS                0x03  // ±250 dps
#define ICM42688_GYRO_FS_125DPS                0x04  // ±125 dps
#define ICM42688_GYRO_FS_62_5DPS               0x05  // ±62.5 dps
#define ICM42688_GYRO_FS_31_25DPS              0x06  // ±31.25 dps

// 数据输出频率(ODR)
#define ICM42688_ODR_32KHZ_LN                  0x01  // 32 kHz in low-noise mode
#define ICM42688_ODR_16KHZ_LN                  0x02  // 16 kHz
#define ICM42688_ODR_8KHZ_LN                   0x03  // 8 kHz
#define ICM42688_ODR_4KHZ_LN                   0x04  // 4 kHz
#define ICM42688_ODR_2KHZ_LN                   0x05  // 2 kHz
#define ICM42688_ODR_1KHZ_LN                   0x06  // 1 kHz
#define ICM42688_ODR_200HZ_LN                  0x07  // 200 Hz
#define ICM42688_ODR_100HZ_LN                  0x08  // 100 Hz
#define ICM42688_ODR_50HZ_LN                   0x09  // 50 Hz
#define ICM42688_ODR_25HZ_LN                   0x0A  // 25 Hz
#define ICM42688_ODR_12_5HZ_LN                 0x0B  // 12.5 Hz
#define ICM42688_ODR_6_25HZ_LN                 0x0C  // 6.25 Hz
#define ICM42688_ODR_3_125HZ_LN                0x0D  // 3.125 Hz
#define ICM42688_ODR_1_5625HZ_LN               0x0E  // 1.5625 Hz
#define ICM42688_ODR_500HZ_LN                  0x0F  // 500 Hz (not in LN mode)

// FIFO 配置
#define ICM42688_FIFO_CONFIG_STREAM            (1 << 7)
#define ICM42688_FIFO_CONFIG_TMST_FSYNC        (1 << 6)
#define ICM42688_FIFO_CONFIG_WM_GT_TH          (1 << 5)
#define ICM42688_FIFO_CONFIG_HIRES_EN          (1 << 4)
#define ICM42688_FIFO_CONFIG_TEMP_EN           (1 << 2)
#define ICM42688_FIFO_CONFIG_GYRO_EN           (1 << 1)
#define ICM42688_FIFO_CONFIG_ACCEL_EN          (1 << 0)

//软复位配置
#define ICM42688_SOFT_RESET_MSK 0x01


// 传感器模式组合
#define ICM42688_MODE_OFF              0x00  // 全部关闭
#define ICM42688_MODE_LN_ACCEL_ONLY    0x03  // 仅加速度计低噪声模式 (0000 0011)
#define ICM42688_MODE_LN_GYRO_ONLY     0x0C  // 仅陀螺仪低噪声模式   (0000 1100)
#define ICM42688_MODE_LN_BOTH          0x0F  // 加速度计+陀螺仪低噪声 (0000 1111)
#define ICM42688_MODE_LP_BOTH          0x06  // 加速度计+陀螺仪低功耗 (0000 0110)

// AAF @ 258Hz (ICM-42688-P datasheet section 5.3, Betaflight GYRO_HARDWARE_LPF_NORMAL)
#define ICM42688_AAF_DELT                  6u
#define ICM42688_AAF_DELTSQR               36u
#define ICM42688_AAF_BITSHIFT              10u
#define ICM42688_GYRO_AAF_STATIC5          ((uint8_t)(((ICM42688_AAF_DELTSQR >> 8) & 0x0Fu) | (ICM42688_AAF_BITSHIFT << 4)))
#define ICM42688_ACCEL_AAF_STATIC2         ((uint8_t)(ICM42688_AAF_DELT << 1))

// UI filter: 2nd order; BW code 5 = max(400Hz, ODR)/16 (at 4kHz ODR, 3dB ~1048Hz per datasheet)
#define ICM42688_GYRO_UI_FILT_2ND_ORD      0x16u  // TEMP default + UI 2nd + DEC2 3rd (reset-like)
#define ICM42688_ACCEL_UI_FILT_2ND_ORD     0x0Du  // UI 2nd + DEC2 3rd (reset-like)
#define ICM42688_UI_FILT_BW_ODR16          0x55u  // accel BW=5, gyro BW=5

#define ICM42688_INTF_CONFIG1_AFSR_MASK    0xC0u
#define ICM42688_INTF_CONFIG1_AFSR_DISABLE 0x40u

#define ICM42688_INT_CONFIG0_DRDY_CLEAR    0x00u
#define ICM42688_INT_CONFIG1_4KHZ_ODR      0x60u  // 8us pulse + de-assert disabled


//时间戳配置
#define ICM42688_TMST_CONFIG         0x54  //bank1
#define ICM42688_TMST_ENABLE         0x11 //bit1 :1(使能) bit2:0(分辨率10us)bit4: 1(时间更新)
//bank0
#define ICM42688_TMSTVAL0                      0x62
#define ICM42688_TMSTVAL1                      0x63
#define ICM42688_TMSTVAL2                      0x64
//=================================================================================================================//




//BANK0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                 0x14
#define ICM42688_FIFO_CONFIG                0x16
#define ICM42688_TMST_FSYNCH                0x2B
#define ICM42688_TMST_FSYNCL                0x2C 
#define ICM42688_INT_STATUS                 0x2D
#define ICM42688_FIFO_COUNTH                0x2E
#define ICM42688_FIFO_COUNTL                0x2F
#define ICM42688_FIFO_DATA                  0x30
#define ICM42688_APEX_DATA0                 0x31
#define ICM42688_APEX_DATA1                 0x32
#define ICM42688_APEX_DATA2                 0x33
#define ICM42688_APEX_DATA3                 0x34
#define ICM42688_APEX_DATA4                 0x35
#define ICM42688_APEX_DATA5                 0x36
#define ICM42688_INT_STATUS2                0x37
#define ICM42688_INT_STATUS3                0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0               0x4C
#define ICM42688_INTF_CONFIG1               0x4D
#define ICM42688_APEX_CONFIG0               0x56
#define ICM42688_SMD_CONFIG                 0x57
#define ICM42688_FIFO_CONFIG1               0x5F
#define ICM42688_FIFO_CONFIG2               0x60
#define ICM42688_FIFO_CONFIG3               0x61
#define ICM42688_FSYNC_CONFIG               0x62
#define ICM42688_INT_CONFIG0                0x63
#define ICM42688_INT_CONFIG1                0x64
#define ICM42688_INT_SOURCE0                0x65
#define ICM42688_INT_SOURCE1                0x66
#define ICM42688_INT_SOURCE3                0x68
#define ICM42688_INT_SOURCE4                0x69    
#define ICM42688_FIFO_LOST_PKT0             0x6C
#define ICM42688_FIFO_LOST_PKT1             0x6D
#define ICM42688_SELF_TEST_CONFIG           0x70

//BANK1
#define ICM42688_SENSOR_CONFIG0               0x03
#define ICM42688_GYRO_CONFIG_STATIC2          0x0B
#define ICM42688_GYRO_CONFIG_STATIC3          0x0C
#define ICM42688_GYRO_CONFIG_STATIC4          0x0D
#define ICM42688_GYRO_CONFIG_STATIC5          0x0E
#define ICM42688_GYRO_CONFIG_STATIC6          0x0F
#define ICM42688_GYRO_CONFIG_STATIC7          0x10
#define ICM42688_GYRO_CONFIG_STATIC8          0x11
#define ICM42688_GYRO_CONFIG_STATIC9          0x12
#define ICM42688_GYRO_CONFIG_STATIC10         0x13
#define ICM42688_XG_ST_DATA                    0x5F
#define ICM42688_YG_ST_DATA                    0x60
#define ICM42688_ZG_ST_DATA                    0x61
#define ICM42688_INTF_CONFIG4                  0x7A
#define ICM42688_INTF_CONFIG5                  0x7B
#define ICM42688_INTF_CONFIG6                  0x7C



//BANK2
#define ICM42688_ACCEL_CONFIG_STATIC2         0x03
#define ICM42688_ACCEL_CONFIG_STATIC3         0x04
#define ICM42688_ACCEL_CONFIG_STATIC4         0x05
#define ICM42688_XA_ST_DATA                    0x3B
#define ICM42688_YA_ST_DATA                    0x3C
#define ICM42688_ZA_ST_DATA                    0x3D


//BANK4
#define ICM42688_APEX_CONFIG1                 0x40
#define ICM42688_APEX_CONFIG2                 0x41
#define ICM42688_APEX_CONFIG3                 0x42
#define ICM42688_APEX_CONFIG4                 0x43
#define ICM42688_APEX_CONFIG5                 0x44
#define ICM42688_APEX_CONFIG6                 0x45
#define ICM42688_APEX_CONFIG7                 0x46
#define ICM42688_APEX_CONFIG8                 0x47
#define ICM42688_APEX_CONFIG9                 0x48
#define ICM42688_ACCEL_WOM_X_THR              0x4A
#define ICM42688_ACCEL_WOM_Y_THR              0x4B
#define ICM42688_ACCEL_WOM_Z_THR              0x4C
#define ICM42688_INT_SOURCE6                  0x4D
#define ICM42688_INT_SOURCE7                  0x4E
#define ICM42688_INT_SOURCE8                  0x4F
#define ICM42688_INT_SOURCE9                  0x50
#define ICM42688_INT_SOURCE10                 0x51
#define ICM42688_OFFSET_USER0                 0x77
#define ICM42688_OFFSET_USER1                 0x78
#define ICM42688_OFFSET_USER2                 0x79
#define ICM42688_OFFSET_USER3                 0x7A
#define ICM42688_OFFSET_USER4                 0x7B
#define ICM42688_OFFSET_USER5                 0x7C
#define ICM42688_OFFSET_USER6                 0x7D
#define ICM42688_OFFSET_USER7                 0x7E
#define ICM42688_OFFSET_USER8                 0x7F


//寄存器长度宏
#define ICM42688_DATA_LEN                    15
#define ICM42688_TIME_LEN                    3


// 最终使用的 Scale
#define ICM42688_TMST_MAX                     1048576     //2^20
#define ICM42688_TMST_RES                     1e-5f  //10us（寄存器中时间戳分辨率为10us）
#define ICM42688_COLLEDCT_TIMES               2000

#endif

