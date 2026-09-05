#ifndef __BMM150_REG_H
#define __BMM150_REG_H

//=============================================================//
//========所有位运算和位移操作请参考数据手册==================//

#define BMM150_CHIP_ID_VALUE          0x32

// BMM150工作模式定义
#define BMM150_NORMAL_MODE      0x00 //连续采样
#define BMM150_FORCED_MODE      0x01 //单次采样后休眠
#define BMM150_SLEEP_MODE       0x03 //休眠（可被唤醒）
#define BMM150_SUSPEND_MODE     0x04 //全死

// 数据速率定义
#define BMM150_DATA_RATE_10HZ   0x00
#define BMM150_DATA_RATE_2HZ    0x01
#define BMM150_DATA_RATE_6HZ    0x02
#define BMM150_DATA_RATE_8HZ    0x03
#define BMM150_DATA_RATE_15HZ   0x04
#define BMM150_DATA_RATE_20HZ   0x05
#define BMM150_DATA_RATE_25HZ   0x06
#define BMM150_DATA_RATE_30HZ   0x07


// 重复测量次数定义（影响精度和功耗）
#define BMM150_REPXY_LOW        0x01   // 1次
#define BMM150_REPXY_REGULAR    0x04   // 9次
#define BMM150_REPXY_ENHANCED   0x07   // 15次
#define BMM150_REPXY_HIGH       0x17   // 47次

#define BMM150_REPZ_LOW         0x01   // 低精度
#define BMM150_REPZ_REGULAR     0x04   // 常规精度
#define BMM150_REPZ_ENHANCED    0x07   //增强精度
#define BMM150_REPZ_HIGH        0x31   // 高精度


// BMM150寄存器地址
#define BMM150_REG_CHIP_ID       0x40
#define BMM150_REG_DATA_X_LSB    0x42 //0-4
#define BMM150_REG_DATA_X_MSB    0x43 //5-12
#define BMM150_REG_DATA_Y_LSB    0x44 //0-4
#define BMM150_REG_DATA_Y_MSB    0x45 //5-12
#define BMM150_REG_DATA_Z_LSB    0x46 //0-6
#define BMM150_REG_DATA_Z_MSB    0x47 //7-14
#define BMM150_REG_DATA_STATUS   0x48
#define BMM150_REG_DATA_READY_STATUS 0x48
#define BMM150_DATA_RHALL_L       0x48 //0-5
#define BMM150_DATA_RHALL_H       0x49 //6-13
#define BMM150_REG_POWER_CONTROL 0x4B
#define BMM150_REG_OP_MODE       0x4C
#define BMM150_REG_INT_CONFIG    0x4D
#define BMM150_REG_AXIS_ENABLE   0x4E
#define BMM150_REG_REP_XY        0x51
#define BMM150_REG_REP_Z         0x52

#define BMM150_DATA_SHIFTX 3
#define BMM150_DATA_SHIFTY 3
#define BMM150_DATA_SHIFTZ 1
#define BMM150_DATA_SHIFT_RHALL 2

#define BMM150_TRIM_REGISTERS_START    0x5D
#define BMM150_TRIM_REGISTER_END        0x71
#define BMM150_TRIM_REGISTERS_LENGTH    21

#define BMM150_TRIM_X1            0x5D
#define BMM150_TRIM_Y1            0x5E
#define BMM150_TRIM_Z4_LSB        0x62
#define BMM150_TRIM_Z4_MSB        0x63
#define BMM150_TRIM_X2             0x64
#define BMM150_TRIM_Y2             0x65
#define BMM150_TRIM_Z2_LSB         0x68
#define BMM150_TRIM_Z2_MSB         0x69
#define BMM150_TRIM_Z1_LSB         0x6A
#define BMM150_TRIM_Z1_MSB         0x6B
#define BMM150_TRIM_XYZ1_LSB       0x6C
#define BMM150_TRIM_XYZ1_MSB       0x6D
#define BMM150_TRIM_Z3_LSB         0x6E
#define BMM150_TRIM_Z3_MSB         0x6F
#define BMM150_TRIM_XY2            0x70
#define BMM150_TRIM_XY1            0x71

#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        INT16_C(-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL         INT16_C(-16384)
#define BMM150_OVERFLOW_OUTPUT                    INT16_C(-32768)
#define BMM150_NEGATIVE_SATURATION_Z              INT16_C(-32767)
#define BMM150_POSITIVE_SATURATION_Z              INT16_C(32767)
#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        INT16_C(-4096)



//算法宏
#define BMM150_TIMEOUT_US               200000
#define BMM150_DATAREGS_LEN             8

#endif
