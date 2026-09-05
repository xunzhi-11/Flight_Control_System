#include "bmp390l/bmp390l.h"
#include "bmp390l/bmp390l_reg.h"
#include "iic_device.h"
#include "sys_hardware_config.h"
#include "data_structure_types.h"
#include "tim/tim5.h"

typedef struct 
{
    // 解析后的浮点系数
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
    double t_lin; // 过程变量
} BMP390L_CALIB_DATA;

typedef struct  
{
    uint8_t bmp390l_rx_buf[BMP390L_DATA_LEN] ; 
    BAROMETER_Raw_t bmp390l_raw_data ; 
    BAROMETER_Real_t bmp390l_real_data ; 
    BMP390L_CALIB_DATA bmp390l_calib_data ; 
    void (*bmp390l_delay)(uint32_t ms) ;
}BMP390L ;

static void bmp390l_delay(uint32_t ms) ; 
static uint8_t bmp390l_read_reg(uint8_t reg) ; 
static void bmp390l_read_regs(uint8_t reg , uint8_t* buf , uint8_t buf_len) ; 
static void bmp390l_write_reg(uint8_t reg , uint8_t data) ; 
static void BMP390L_EnableDRDY_Interrupt(void) ; 
static void BMP390_Read_And_Convert_Calib_Data(BMP390L_CALIB_DATA *calib) ; 
static float BMP390L_Get_Temp(BAROMETER_Raw_t *raw, BMP390L_CALIB_DATA *calib) ; 
static float BMP390L_Get_Press(BAROMETER_Raw_t *raw, BMP390L_CALIB_DATA *calib) ; 
static uint8_t BMP390L_GetChipID(void) ; 

static volatile uint32_t bmp390l_seq_lock = 0 ; 
static BMP390L bmp390l = {0} ; 

IIC_Device_t bmp390l_hw = 
{
    .I2Cx = HW_I2C_BMP390L_INSTANCE , 
    .DevAddress = HW_I2C_BMP390L_ADDR_8BIT , 
    .rx_callback = bmp390l_dma_rx_complete_callback 
} ; 

static void bmp390l_delay(uint32_t ms)
{
    if (bmp390l.bmp390l_delay != NULL) {
        bmp390l.bmp390l_delay(ms) ;
    } else {
        // 防宕机死等兜底 (168MHz)
        volatile uint32_t i = ms * 20000 ;
        while(i--) { __NOP() ; }
    }
}

static uint8_t bmp390l_read_reg(uint8_t reg)
{
    return IIC_Dev_ReadReg(&bmp390l_hw , reg) ; 
}

static void bmp390l_read_regs(uint8_t reg , uint8_t* buf , uint8_t buf_len)
{
    IIC_Dev_ReadRegs(&bmp390l_hw , reg , buf , buf_len) ; 
}

static void bmp390l_write_reg(uint8_t reg , uint8_t data)
{
    IIC_Dev_WriteReg(&bmp390l_hw , reg , data) ; 
}

static void BMP390L_EnableDRDY_Interrupt(void)
{
    bmp390l_write_reg(BMP390L_INT_CTRL, 0x43);
}

static void BMP390_Read_And_Convert_Calib_Data(BMP390L_CALIB_DATA *calib)
{
    uint8_t reg_data[21] = {0};
    bmp390l_read_regs(BMP390L_NVM_PAR_T1_L, reg_data, 21);

    // --- 临时变量 (取出原始整数) ---
    uint16_t nvm_t1 = (uint16_t)(reg_data[1] << 8 | reg_data[0]);
    uint16_t nvm_t2 = (uint16_t)(reg_data[3] << 8 | reg_data[2]);
    int8_t   nvm_t3 = (int8_t)reg_data[4];

    int16_t nvm_p1 = (int16_t)(reg_data[6] << 8 | reg_data[5]);
    int16_t nvm_p2 = (int16_t)(reg_data[8] << 8 | reg_data[7]);
    int8_t  nvm_p3 = (int8_t)reg_data[9];
    int8_t  nvm_p4 = (int8_t)reg_data[10];
    uint16_t nvm_p5 = (uint16_t)(reg_data[12] << 8 | reg_data[11]);
    uint16_t nvm_p6 = (uint16_t)(reg_data[14] << 8 | reg_data[13]);
    int8_t  nvm_p7 = (int8_t)reg_data[15];
    int8_t  nvm_p8 = (int8_t)reg_data[16];
    int16_t nvm_p9 = (int16_t)(reg_data[18] << 8 | reg_data[17]);
    int8_t  nvm_p10 = (int8_t)reg_data[19];
    int8_t  nvm_p11 = (int8_t)reg_data[20];

    // 转换为物理浮点系数 
    calib->par_t1 = (float)(nvm_t1 / 0.00390625f);
    calib->par_t2 = (float)(nvm_t2 / 1073741824.0f);
    calib->par_t3 = (float)(nvm_t3 / 281474976710656.0f);

    calib->par_p1 = (float)(nvm_p1 - 16384) / 1048576.0f;
    calib->par_p2 = (float)(nvm_p2 - 16384) / 536870912.0f;
    calib->par_p3 = (float)nvm_p3 / 4294967296.0f;
    calib->par_p4 = (float)nvm_p4 / 137438953472.0f;
    calib->par_p5 = (float)nvm_p5 / 0.125f;
    calib->par_p6 = (float)nvm_p6 / 64.0f;
    calib->par_p7 = (float)nvm_p7 / 256.0f;
    calib->par_p8 = (float)nvm_p8 / 32768.0f;
    calib->par_p9 = (float)nvm_p9 / 281474976710656.0f;
    calib->par_p10 = (float)nvm_p10 / 281474976710656.0f;
    calib->par_p11 = (float)nvm_p11 / 36893488147419103232.0f;
}

static float BMP390L_Get_Temp(BAROMETER_Raw_t *raw, BMP390L_CALIB_DATA *calib)
{
    // 强制把转换结果拉到 FPU 寄存器
    float raw_t = (float)raw->temp; 
    
    float partial_data1 = raw_t - calib->par_t1;

    calib->t_lin = partial_data1 * (calib->par_t2 + partial_data1 * calib->par_t3);

    return calib->t_lin;
}

static float BMP390L_Get_Press(BAROMETER_Raw_t *raw, BMP390L_CALIB_DATA *calib)
{
    // 本地缓存，强迫编译器使用 FPU 寄存器 (S 寄存器)
    float t = calib->t_lin; 
    float p = (float)raw->press; 

    // 秦九韶算法展开，直接触发编译器的连串 VMLA.F32 乘加指令
    float out1 = calib->par_p5 + t * (calib->par_p6 + t * (calib->par_p7 + t * calib->par_p8));

    // 压强多项式 2: p1 + p2*t + p3*t^2 + p4*t^3
    float out2_coeff = calib->par_p1 + t * (calib->par_p2 + t * (calib->par_p3 + t * calib->par_p4));
    float out2 = p * out2_coeff;

    // 高阶补偿多项式: p^2 * (p9 + p10*t) + p^3 * p11
    // 提取 p^2 因子: p^2 * (p9 + p10*t + p * p11)
    float p2 = p * p;
    float out3 = p2 * (calib->par_p9 + calib->par_p10 * t + p * calib->par_p11);

    return out1 + out2 + out3;
}

static uint8_t BMP390L_GetChipID(void)
{ 
    return bmp390l_read_reg(BMP390L_CHIP_ID_ADDR);  
}

void bmp390l_drdy_exti_handler(void) 
{
    if (IIC_Dev_Is_Busy(DEVICE_IIC2)) 
    {
        return; 
    }

    IIC_Dev_ReadRegs_DMA(&bmp390l_hw , BMP390L_PRESS_L , bmp390l.bmp390l_rx_buf , BMP390L_DATA_LEN) ; 
    bmp390l.bmp390l_raw_data.god_time = TIM5_GetStamp() ;  
}

void bmp390l_dma_rx_complete_callback(void)
{
    bmp390l.bmp390l_raw_data.press = (uint32_t)((bmp390l.bmp390l_rx_buf[2] << 16) | (bmp390l.bmp390l_rx_buf[1] << 8) | bmp390l.bmp390l_rx_buf[0]); 
    bmp390l.bmp390l_raw_data.temp  = (uint32_t)((bmp390l.bmp390l_rx_buf[5] << 16) | (bmp390l.bmp390l_rx_buf[4] << 8) | bmp390l.bmp390l_rx_buf[3]); 

    float temp_temp = BMP390L_Get_Temp(&bmp390l.bmp390l_raw_data , &bmp390l.bmp390l_calib_data) ; 
    float press_temp = BMP390L_Get_Press(&bmp390l.bmp390l_raw_data , &bmp390l.bmp390l_calib_data) ; 

    bmp390l_seq_lock ++ ; 
    __DMB() ; 

    bmp390l.bmp390l_real_data.press = press_temp ; 
    bmp390l.bmp390l_real_data.temp = temp_temp ; 
    bmp390l.bmp390l_real_data.god_time = bmp390l.bmp390l_raw_data.god_time ; 

    __DMB() ; 
    bmp390l_seq_lock ++ ; 

}

void BMP390l_Register_Delay(void (*delay_func)(uint32_t ms))
{
    bmp390l.bmp390l_delay = delay_func ;
}

void bmp390l_init(void)
{
    
    bmp390l_write_reg(BMP390L_CMD, 0xB6); 
    bmp390l_delay(20) ;

    if(BMP390L_GetChipID() != BMP390L_CHIP_ID_VALUE) 
    {
        return ; 
    }

    bmp390l_write_reg(BMP390L_OSR , 0x03); 
    bmp390l_delay(25) ; 
    bmp390l_write_reg(BMP390L_ODR , 0x03);
    bmp390l_delay(25) ; 
    bmp390l_write_reg(BMP390L_CONFIG , 0x02) ; 
    bmp390l_delay(25) ;

    BMP390_Read_And_Convert_Calib_Data(&bmp390l.bmp390l_calib_data) ;

    bmp390l_delay(50) ; 

    bmp390l_write_reg(BMP390L_PWR_CTRL , 0x33); 

    BMP390L_EnableDRDY_Interrupt() ; 
    bmp390l_delay(50) ; 
}

void bmp390l_get_real_data(BAROMETER_Real_t* out_data)
{
    uint32_t seq ;

    if(out_data == NULL) return ; 

    do {
        seq = bmp390l_seq_lock ;
        
        while (seq & 1) 
        {
            seq = bmp390l_seq_lock ; 
        }
        
        COMPILER_BARRIER() ; 

        *out_data = bmp390l.bmp390l_real_data ;
        
        COMPILER_BARRIER() ;

    } while (seq != bmp390l_seq_lock) ; 
}
