#include "bmm150/bmm150.h"
#include "bmm150/bmm150_reg.h"
#include "spi_device.h"
#include "sys_hardware_config.h"
#include "tim/tim5.h"

typedef struct 
{
    int8_t trim_x1 ; 
    int8_t trim_y1 ; 
    int8_t trim_x2 ; 
    int8_t trim_y2 ; 
    int8_t trim_xy1 ;
    int8_t trim_xy2 ;
    int16_t trim_z1 ;
    int16_t trim_z2 ;
    int16_t trim_z3 ;
    int16_t trim_z4 ;
    uint16_t trim_xyz1 ; 
} Trim ;

// 原始数据结构体
typedef struct 
{
    int16_t raw_x ;     
    int16_t raw_y ;     
    int16_t raw_z ;      
    uint16_t rhall ;     
} BMM150_RawData ;

struct BMM150
{
    __attribute__((aligned(4))) uint8_t bmm150_rx_buffer[12] ;
    __attribute__((aligned(4))) uint8_t bmm150_tx_buffer[12] ; 
    BMM150_RawData bmm150_rawdata ; 
    MAGNETOMETER_Trimed_t bmm150_trimed_data ; 
    Trim trim ; 
    void (*bmm150_delay)(uint32_t ms) ;
} ;

static struct BMM150 bmm150 = {0} ;
volatile uint32_t bmm150_seq_lock = 0 ;


static void bmm150_write_reg(uint8_t reg , uint8_t data) ; 
static uint8_t bmm150_read_reg(uint8_t reg) ; 
static void bmm150_read_regs(uint8_t reg , uint8_t* buf , uint8_t buf_len) ; 
static void Get_Trim_Data(Trim* trim) ; 
static float compensate_x(int16_t mag_data_x ,  Trim* trim , uint16_t data_rhall) ; 
static float compensate_y(int16_t mag_data_y ,  Trim* trim , uint16_t data_rhall) ; 
static float compensate_z(int16_t mag_data_z ,  Trim* trim , uint16_t data_rhall) ; 
static float compensate_x_from_raw(BMM150_RawData* raw_data, Trim* trim) ; 
static float compensate_y_from_raw(BMM150_RawData* raw_data, Trim* trim) ; 
static float compensate_z_from_raw(BMM150_RawData* raw_data, Trim* trim) ; 
static void bmm150_dma_rx_complete_callback(void) ;
static void BMM150_Speed_Change(uint8_t Speed) ;  
static void BMM150_Acurrancy_Change(uint8_t xy_acr , uint8_t z_acr) ; 
static void BMM150_EnableDRDY_Interrupt(void) ; 
static void bmm150_delay(uint32_t ms) ; 

SPI_Device_t bmm150_hw = 
{
    .SPIx    = HW_SPI_BMM150_INSTANCE,
    .CS_Port = HW_GPIO_BMM150_CS_PORT,
    .CS_Pin  = HW_GPIO_BMM150_CS_PIN , 
    .rx_callback = bmm150_dma_rx_complete_callback,
} ; 

static void bmm150_write_reg(uint8_t reg , uint8_t data)
{
    SPI_Dev_WriteReg(&bmm150_hw , reg , data) ; 
}

static uint8_t bmm150_read_reg(uint8_t reg)
{
    return SPI_Dev_ReadReg(&bmm150_hw , reg) ; 
}

static void bmm150_read_regs(uint8_t reg , uint8_t* buf , uint8_t buf_len)
{
    SPI_Dev_ReadRegs(&bmm150_hw , reg , buf , buf_len) ; 
}

static void Get_Trim_Data(Trim* trim)
{
    uint8_t buffer[BMM150_TRIM_REGISTERS_LENGTH] ;
    
    bmm150_read_regs(BMM150_TRIM_REGISTERS_START, buffer, BMM150_TRIM_REGISTERS_LENGTH) ;

    trim->trim_x1   = (int8_t)buffer[0] ; 
    trim->trim_y1   = (int8_t)buffer[1] ;
    trim->trim_x2   = (int8_t)buffer[7] ;
    trim->trim_y2   = (int8_t)buffer[8] ;
    trim->trim_xy2  = (int8_t)buffer[19] ;
    trim->trim_xy1  = (int8_t)buffer[20] ; 
    trim->trim_z4   = (int16_t)((int16_t)buffer[6] << 8 | buffer[5]) ;
    trim->trim_xyz1 = (int16_t)((uint16_t)buffer[16] << 8 | buffer[15]) ; 
    trim->trim_z1   = (int16_t)((int16_t)buffer[14] << 8 | buffer[13]) ;
    trim->trim_z2   = (int16_t)((int16_t)buffer[12] << 8 | buffer[11]) ;
    trim->trim_z3   = (int16_t)((int16_t)buffer[18] << 8 | buffer[17]) ;
}

static void BMM150_Speed_Change(uint8_t Speed)
{
    uint8_t op_mode_config = bmm150_read_reg(BMM150_REG_OP_MODE) ;

    /* 手册：改 ODR 时应处于 Sleep；bits2-1=11 为 Sleep，bits5-3 为 Data rate */
    op_mode_config &= 0xC1u ;
    op_mode_config |= (uint8_t)((Speed << 3) | (0x03u << 1)) ;

    bmm150_write_reg(BMM150_REG_OP_MODE, op_mode_config) ;
}
 
static void BMM150_Acurrancy_Change(uint8_t xy_acr , uint8_t z_acr)
{
    bmm150_write_reg(BMM150_REG_REP_XY, xy_acr) ;
    bmm150_write_reg(BMM150_REG_REP_Z, z_acr) ;
}

static void BMM150_EnableDRDY_Interrupt(void)
{
    /* 与老工程一致：整字节写入 0x4E（DRDY+INT 引脚使能、轴开、极性/锁存位与默认 0x07 低位对齐为 0xC7） */
    uint8_t reg_4e = 0xC7u ;
    bmm150_write_reg(BMM150_REG_AXIS_ENABLE, reg_4e) ;
}

static float compensate_x(int16_t mag_data_x ,  Trim* trim , uint16_t data_rhall)
{
     
    float retval = 0 ;
    float process_comp_x0 ;
    float process_comp_x1 ;
    float process_comp_x2 ;
    float process_comp_x3 ;
    float process_comp_x4 ;
   
    /* Overflow condition check */
    if ((mag_data_x != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) && (data_rhall != 0) && (trim->trim_xyz1 != 0))
    {
        /* Processing compensation equations */
        process_comp_x0 = (((float)trim->trim_xyz1) * 16384.0f / data_rhall) ;
        retval = (process_comp_x0 - 16384.0f) ;
        process_comp_x1 = ((float)trim->trim_xy2) * (retval * retval / 268435456.0f) ;
        process_comp_x2 = process_comp_x1 + retval * ((float)trim->trim_xy1) / 16384.0f ;
        process_comp_x3 = ((float)trim->trim_x2) + 160.0f ;
        process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3) ;
        retval = ((process_comp_x4 / 8192.0f) + (((float)trim->trim_x1) * 8.0f)) / 16.0f ;
    }
    else
    {
        /* Overflow, set output to 0.0f */
        retval = 0.0f ;
    }

    return retval ;
}

static float compensate_y(int16_t mag_data_y ,  Trim* trim , uint16_t data_rhall)
{
    float retval = 0 ;
    float process_comp_y0 ;
    float process_comp_y1 ;
    float process_comp_y2 ;
    float process_comp_y3 ;
    float process_comp_y4 ;
    
    /* Overflow condition check */
    if ((mag_data_y != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) && (data_rhall != 0) && (trim->trim_xyz1 != 0))
    {
        /* Processing compensation equations */
        process_comp_y0 = ((float)trim->trim_xyz1) * 16384.0f / data_rhall ;
        retval = process_comp_y0 - 16384.0f ;
        process_comp_y1 = ((float)trim->trim_xy2) * (retval * retval / 268435456.0f) ;
        process_comp_y2 = process_comp_y1 + retval * ((float)trim->trim_xy1) / 16384.0f ;
        process_comp_y3 = ((float)trim->trim_y2) + 160.0f ;
        process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3) ;
        retval = ((process_comp_y4 / 8192.0f) + (((float)trim->trim_y1) * 8.0f)) / 16.0f ;
    }
    else
    {
        /* Overflow, set output to 0.0f */
        retval = 0.0f ;
    }

    return retval ;
}

static float compensate_z(int16_t mag_data_z ,  Trim* trim , uint16_t data_rhall)
{
    float retval = 0 ;
    float process_comp_z0 ;
    float process_comp_z1 ;
    float process_comp_z2 ;
    float process_comp_z3 ;
    float process_comp_z4 ;
    float process_comp_z5 ;
   
    /* Overflow condition check */
    if ((mag_data_z != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL) && (trim->trim_z2 != 0) &&
        (trim->trim_z1 != 0) && (trim->trim_xyz1 != 0) && (data_rhall != 0))
    {
        /* Processing compensation equations */
        process_comp_z0 = ((float)mag_data_z) - ((float)trim->trim_z4) ;
        process_comp_z1 = ((float)data_rhall) - ((float)trim->trim_xyz1) ;
        process_comp_z2 = (((float)trim->trim_z3) * process_comp_z1) ;
        process_comp_z3 = ((float)trim->trim_z1) * ((float)data_rhall) / 32768.0f ;
        process_comp_z4 = ((float)trim->trim_z2) + process_comp_z3 ;
        process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2 ;
        retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f ;
    }
    else
    {
        /* Overflow, set output to 0.0f */
        retval = 0.0f ;
    }

    return retval ;
}

//补偿封装
static float compensate_x_from_raw(BMM150_RawData* raw_data, Trim* trim)
{
    return compensate_x(raw_data->raw_x, trim, raw_data->rhall) ;
}

static float compensate_y_from_raw(BMM150_RawData* raw_data, Trim* trim)
{
    return compensate_y(raw_data->raw_y, trim, raw_data->rhall) ;
}

static float compensate_z_from_raw(BMM150_RawData* raw_data, Trim* trim)
{
    return compensate_z(raw_data->raw_z, trim, raw_data->rhall) ;
}

static void bmm150_dma_rx_complete_callback(void)
{ 
    uint8_t *buf = bmm150.bmm150_rx_buffer ;

    bmm150.bmm150_rawdata.raw_x = (int16_t)((buf[2] << 8) | buf[1]) >> BMM150_DATA_SHIFTX ;
    bmm150.bmm150_rawdata.raw_y = (int16_t)((buf[4] << 8) | buf[3]) >> BMM150_DATA_SHIFTY ;
    bmm150.bmm150_rawdata.raw_z = (int16_t)((buf[6] << 8) | buf[5]) >> BMM150_DATA_SHIFTZ ;
    bmm150.bmm150_rawdata.rhall = (uint16_t)((buf[8] << 8) | buf[7]) >> BMM150_DATA_SHIFT_RHALL ;
    
    float temp_x = compensate_x_from_raw(&bmm150.bmm150_rawdata, &bmm150.trim) ;
    float temp_y = compensate_y_from_raw(&bmm150.bmm150_rawdata, &bmm150.trim) ;
    float temp_z = compensate_z_from_raw(&bmm150.bmm150_rawdata, &bmm150.trim) ;
    // ================= 序列锁：写入临界区开始 =================
    bmm150_seq_lock++ ; // 变为奇数
    __DMB() ;           // 数据内存屏障，严禁编译器或 CPU 将下面的代码乱序重排到上面

    bmm150.bmm150_trimed_data.mag_x = temp_x ;
    bmm150.bmm150_trimed_data.mag_y = temp_y ;
    bmm150.bmm150_trimed_data.mag_z = temp_z ; 

    __DMB() ;           // 数据内存屏障
    bmm150_seq_lock++ ; // 变为偶数
    // ================= 序列锁：写入临界区结束 =================
}

static void bmm150_delay(uint32_t ms)
{
    if (bmm150.bmm150_delay != NULL) {
        bmm150.bmm150_delay(ms) ;
    } else {
        // 防宕机死等兜底 (168MHz)
        volatile uint32_t i = ms * 20000 ;
        while(i--) { __NOP() ; }
    }
}

void BMM150_Register_Delay(void (*delay_func)(uint32_t ms))
{
    bmm150.bmm150_delay = delay_func ;
}

void bmm150_drdy_exti_handler(void)
{
    bmm150.bmm150_trimed_data.god_time = TIM5_GetStamp() ;
    SPI_Dev_ReadRegs_DMA(&bmm150_hw, bmm150.bmm150_tx_buffer, bmm150.bmm150_rx_buffer, BMM150_DATAREGS_LEN + 1);
}

uint8_t BMM150_Get_ChipID(void)
{
    return bmm150_read_reg(BMM150_REG_CHIP_ID) ;
}

void bmm150_get_trimed_data(MAGNETOMETER_Trimed_t* out_data)
{
    uint32_t seq ;

    if(out_data == NULL) return ; 

    do {
        // 获取当前版本号
        seq = bmm150_seq_lock ;
        
        // 如果是奇数，说明 ISR 正在写。
        // 由于是单核，如果当前在任务里，说明 ISR 刚刚退出了（因为任务能跑，说明没有 ISR 在跑）
        // 此时遇到奇数只有一种极其罕见的情况：ISR 在临界区内崩溃了
        // 但安全起见，依然加上防奇数逻辑
        if (seq & 1) 
        {
            continue ; 
        }
        
        __DMB() ; // 内存屏障，确保先读版本号，再读数据

        *out_data = bmm150.bmm150_trimed_data ; 
        
        __DMB() ; // 内存屏障，确保数据读完后，再校验一次版本号
        
    //  如果校验发现版本号变了，说明在拷贝的途中，被 DMA TC ISR 打断并刷新了数据，此时直接重新循环再拷一次
    } while (seq != bmm150_seq_lock) ; 
}

void bmm150_init(void)
{
    for(int i = 0 ; i < BMM150_DATAREGS_LEN + 1 ; i++)
    {
        bmm150.bmm150_tx_buffer[i] = 0xFF ; 
    }
    bmm150.bmm150_tx_buffer[0] = BMM150_REG_DATA_X_LSB | 0x80 ;


    bmm150_write_reg(BMM150_REG_POWER_CONTROL, 0x82) ;
    bmm150_delay(10) ; // 等待复位完成

    // 唤醒到 Sleep Mode (Bit0=1)
    bmm150_write_reg(BMM150_REG_POWER_CONTROL, 0x01) ; 
    
    // 等待内部电路建立
    bmm150_delay(10) ;

    if(BMM150_Get_ChipID() != BMM150_CHIP_ID_VALUE)
    {
        return ; 
    }

    // 读取出厂补偿值 
    Get_Trim_Data(&bmm150.trim) ;

    //  配置测量参数 (XY/Z 轴的测量重复次数)
    BMM150_Acurrancy_Change(BMM150_REPXY_REGULAR, BMM150_REPZ_REGULAR) ;
    
    // 配置输出速率 (ODR)
    BMM150_Speed_Change(BMM150_DATA_RATE_30HZ) ; 
    bmm150_delay(5) ; 

    // 切入 Normal Mode 开始连续测量
    // 用先读后写(Read-Modify-Write)确保不覆盖 ODR 等配置
    uint8_t op_reg = bmm150_read_reg(BMM150_REG_OP_MODE) ;
    op_reg &= 0xF9 ;  // 清除 Bit2:1 (Opmode 位)
    op_reg |= 0x00 ;  // 设置为 00 (Normal Mode) 
    bmm150_write_reg(BMM150_REG_OP_MODE, op_reg) ;
    
    bmm150_delay(40) ; 
    uint8_t dummy_buf[8];
    bmm150_read_regs(BMM150_REG_DATA_X_LSB, dummy_buf, 8);
    // bmm150_delay(20) ;
    // bmm150_read_regs(BMM150_REG_DATA_X_LSB, dummy_buf, 8);

    BMM150_EnableDRDY_Interrupt() ;
}
