#ifndef __BMP390L_REGS_H
#define __BMP390L_REGS_H
/*
 *sensor config addr
 */
#define BMP390L_CHIP_ID_ADDR    0x00
#define BMP390L_STATUS           0x03
#define BMP390L_PRESS_L         0x04
#define BMP390L_PRESS_M         0x05
#define BMP390L_PRESS_H         0x06
#define BMP390L_TEMP_L          0x07
#define BMP390L_TEMP_M          0x08
#define BMP390L_TEMP_H          0x09
#define BMP390L_TIME_STAMPLE_L 0x0C
#define BMP390L_TIME_STAMPLE_M 0x0D
#define BMP390L_TIME_STAMPLE_H 0x0E
#define BMP390L_EVENT           0x10
#define BMP390L_INT_STATUS      0x11
#define BMP390L_FIFO_LENGTH_F   0x12
#define BMP390L_FIFO_LENGTH_S   0x13
#define BMP390L_FIFO_DATA       0x14
#define BMP390L_FIFO_WTM_F      0x15
#define BMP390L_FIFO_WTM_S      0x16
#define BMP390L_FIFO_GONFIG_F   0x17
#define BMP390L_FIFO_CONFIG_S   0x18
#define BMP390L_INT_CTRL        0x19
#define BMP390L_IF_CONF         0x1A
#define BMP390L_PWR_CTRL        0x1B
#define BMP390L_OSR             0x1C
#define BMP390L_ODR             0x1D
#define BMP390L_CONFIG          0x1F
#define BMP390L_CMD             0x7E


/*
 *calibrate data addr
 */

#define BMP390L_NVM_PAR_T1_L    0x31
#define BMP390L_NVM_PAR_T1_H    0x32
#define BMP390L_NVM_PAR_T2_L    0x33
#define BMP390L_NVM_PAR_T2_H    0x34
#define BMP390L_NVM_PAR_T3      0x35
#define BMP390L_NVM_PAR_P1_L    0x36
#define BMP390L_NVM_PAR_P1_H    0x37
#define BMP390L_NVM_PAR_P2_L    0x38
#define BMP390L_NVM_PAR_P2_H    0x39
#define BMP390L_NVM_PAR_P3      0x3A
#define BMP390L_NVM_PAR_P4      0x3B
#define BMP390L_NVM_PAR_P5_L    0x3C
#define BMP390L_NVM_PAR_P5_H    0x3D
#define BMP390L_NVM_PAR_P6_L    0x3E
#define BMP390L_NVM_PAR_P6_H    0x3F
#define BMP390L_NVM_PAR_P7      0x40
#define BMP390L_NVM_PAR_P8      0x41
#define BMP390L_NVM_PAR_P9_L    0x42
#define BMP390L_NVM_PAR_P9_H    0x43
#define BMP390L_NVM_PAR_P10      0x44
#define BMP390L_NVM_PAR_P11      0x45




/**\name Temperature range values in integer and float */
#define BMP3_MIN_TEMP_INT                       INT64_C(-4000)
#define BMP3_MAX_TEMP_INT                       INT64_C(8500)
#define BMP3_MIN_TEMP_DOUBLE                    -40.0f
#define BMP3_MAX_TEMP_DOUBLE                    85.0f

/*name Pressure range values in integer and float */
#define BMP3_MIN_PRES_INT                       UINT64_C(3000000)
#define BMP3_MAX_PRES_INT                       UINT64_C(12500000)
#define BMP3_MIN_PRES_DOUBLE                    30000.0f
#define BMP3_MAX_PRES_DOUBLE                    125000.0f


/*name Pressure range values in integer and float */
#define BMP3_MIN_PRES_INT                       UINT64_C(3000000)
#define BMP3_MAX_PRES_INT                       UINT64_C(12500000)
#define BMP3_MIN_PRES_DOUBLE                    30000.0f
#define BMP3_MAX_PRES_DOUBLE                    125000.0f

#define BMP390L_DATA_LEN                        6U

#define BMP390L_CHIP_ID_VALUE                   0x60

#endif
