#include "icm42688/icm42688.h"
#include "icm42688/icm42688_reg.h"
#include "spi_device.h"
#include "sys_hardware_config.h"
#include "data_structure_types.h"
#include "tim/tim5.h"

struct ICM42688
{
    __attribute__((aligned(4))) uint8_t icm42688_rx_rawdata_buf[16] ; 
    __attribute__((aligned(4))) uint8_t icm42688_tx_buffer[16] ; 
    uint32_t exti_time_stamp ; 
    IMU_Raw_t icm42688_raw_data ; 
    void (*icm42688_delay)(uint32_t ms) ;
} ;

static void icm42688_delay(uint32_t ms) ; 
static void icm42688_write_reg(uint8_t reg , uint8_t data) ; 
static uint8_t icm42688_read_reg(uint8_t reg) ; 
static void ICM42688_ChangeBANK(uint8_t BankNum) ; 
static void ICM42688_EnableDRDY_Interrupt(void) ;
static void icm42688_configure_signal_path(void) ;
static void icm42688_dma_rx_complete_callback(void) ; 

SPI_Device_t icm42688_hw = 
{
    .SPIx    = HW_SPI_ICM42688_INSTANCE,
    .CS_Port = HW_GPIO_ICM42688_CS_PORT,
    .CS_Pin  = HW_GPIO_ICM42688_CS_PIN , 
    .rx_callback = icm42688_dma_rx_complete_callback,
} ; 

static struct ICM42688 icm42688 = {0} ; 
volatile uint32_t icm42688_seq_lock = 0 ;
static void (*ahrs_task_trigger_cb)(void) = {NULL} ; 

void icm42688_ahrs_task_trigger_register(void (*callback)(void)) 
{
    ahrs_task_trigger_cb = callback ;
}

static void icm42688_delay(uint32_t ms)
{
    if (icm42688.icm42688_delay != NULL) {
        icm42688.icm42688_delay(ms) ;
    } else {
        // 防宕机死等兜底 (168MHz)
        volatile uint32_t i = ms * 20000 ;
        while(i--) { __NOP() ; }
    }
}

static void icm42688_write_reg(uint8_t reg , uint8_t data)
{
    SPI_Dev_WriteReg(&icm42688_hw , reg , data) ; 
}

static uint8_t icm42688_read_reg(uint8_t reg)
{
    return SPI_Dev_ReadReg(&icm42688_hw , reg) ; 
}

static void ICM42688_ChangeBANK(uint8_t BankNum)
{
    switch (BankNum)
    {
        case 0 :
            icm42688_write_reg(ICM42688_REG_BANK_SEL , ICM42688_BANK0) ;
        break ;
        
        case 1 :
            icm42688_write_reg(ICM42688_REG_BANK_SEL , ICM42688_BANK1) ;
        break ;
        
        case 2 :
            icm42688_write_reg(ICM42688_REG_BANK_SEL , ICM42688_BANK2) ;
        break ;
        
        case 4 :
            icm42688_write_reg(ICM42688_REG_BANK_SEL , ICM42688_BANK4) ;
        break ;
    }
}


static void ICM42688_EnableDRDY_Interrupt(void)
{
    ICM42688_ChangeBANK(0) ;

    // INT_CONFIG: 脉冲 / 推挽 / 高有效
    icm42688_write_reg(ICM42688_INT_CONFIG, 0x03) ;
    icm42688_write_reg(ICM42688_INT_CONFIG0, ICM42688_INT_CONFIG0_DRDY_CLEAR) ;

    // ODR >= 4kHz 需 8us 脉冲且关闭 de-assert (datasheet 14.50)
    uint8_t int_cfg1 = icm42688_read_reg(ICM42688_INT_CONFIG1) ;
    int_cfg1 &= ~(1u << 4) ;
    int_cfg1 |= ICM42688_INT_CONFIG1_4KHZ_ODR ;
    icm42688_write_reg(ICM42688_INT_CONFIG1, int_cfg1) ;

    icm42688_write_reg(ICM42688_INT_SOURCE0, 0x08) ;
}

static void icm42688_configure_signal_path(void)
{
    // 写 Bank1/2 前关闭传感器 (datasheet section 12.9)
    ICM42688_ChangeBANK(0) ;
    icm42688_write_reg(ICM42688_PWR_MGMT0, ICM42688_MODE_OFF) ;
    icm42688_delay(1) ;

    // Gyro AAF @ 258Hz, 保持片上 Notch 默认开启
    ICM42688_ChangeBANK(1) ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC2, 0x00) ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC3, ICM42688_AAF_DELT) ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC4, (uint8_t)(ICM42688_AAF_DELTSQR & 0xFFu)) ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC5, ICM42688_GYRO_AAF_STATIC5) ;

    // Accel AAF @ 258Hz
    ICM42688_ChangeBANK(2) ;
    icm42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, ICM42688_ACCEL_AAF_STATIC2) ;
    icm42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC3, (uint8_t)(ICM42688_AAF_DELTSQR & 0xFFu)) ;
    icm42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC4, ICM42688_GYRO_AAF_STATIC5) ;

    // UI 二阶滤波 + BW=ODR/16
    ICM42688_ChangeBANK(0) ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG1, ICM42688_GYRO_UI_FILT_2ND_ORD) ;
    icm42688_write_reg(ICM42688_ACCEL_CONFIG1, ICM42688_ACCEL_UI_FILT_2ND_ORD) ;
    icm42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, ICM42688_UI_FILT_BW_ODR16) ;

    // 关闭 AFSR，避免 gyro 输出 stall (Betaflight / ArduPilot)
    uint8_t intf_cfg1 = icm42688_read_reg(ICM42688_INTF_CONFIG1) ;
    intf_cfg1 &= (uint8_t)~ICM42688_INTF_CONFIG1_AFSR_MASK ;
    intf_cfg1 |= ICM42688_INTF_CONFIG1_AFSR_DISABLE ;
    icm42688_write_reg(ICM42688_INTF_CONFIG1, intf_cfg1) ;
}

static void icm42688_dma_rx_complete_callback(void)
{
    uint8_t* buf = icm42688.icm42688_rx_rawdata_buf ; 
    
    icm42688_seq_lock ++ ; 
    COMPILER_BARRIER() ;
    
    icm42688.icm42688_raw_data.ax = (int16_t)((buf[3] << 8) | buf[4]) ; 
    icm42688.icm42688_raw_data.ay = (int16_t)((buf[5] << 8) | buf[6]) ; 
    icm42688.icm42688_raw_data.az = (int16_t)((buf[7] << 8) | buf[8]) ; 
    icm42688.icm42688_raw_data.gx = (int16_t)((buf[9] << 8) | buf[10]) ; 
    icm42688.icm42688_raw_data.gy = (int16_t)((buf[11] << 8) | buf[12]) ; 
    icm42688.icm42688_raw_data.gz = (int16_t)((buf[13] << 8) | buf[14]) ; 
    
    icm42688.icm42688_raw_data.irq_timestamp = icm42688.exti_time_stamp;

    COMPILER_BARRIER() ; 
    icm42688_seq_lock ++ ;
    if (ahrs_task_trigger_cb != NULL) 
    {
        ahrs_task_trigger_cb();
    }
}

void icm42688_init(void)
{
    for(int i = 0 ; i < 16 ; i++)
    {
        icm42688.icm42688_tx_buffer[i] = 0xFF ; 
    }
    icm42688.icm42688_tx_buffer[0] = ICM42688_TEMP_DATA1 | 0x80 ; 

    icm42688_write_reg(ICM42688_DEVICE_CONFIG, ICM42688_SOFT_RESET_MSK) ;
    icm42688_delay(50) ;

    ICM42688_ChangeBANK(ICM42688_BANK0) ;
    icm42688_delay(50) ;

    if (icm42688_read_reg(ICM42688_WHO_AM_I) != ICM42688_WHO_AM_I_VALUE)
    {
        return ;
    }
    icm42688_configure_signal_path() ;
    ICM42688_EnableDRDY_Interrupt() ;

    icm42688_write_reg(ICM42688_PWR_MGMT0, ICM42688_MODE_LN_BOTH) ;
    icm42688_delay(15) ;

    uint8_t gyro_config = ICM42688_GYRO_FS_2000DPS | ICM42688_ODR_4KHZ_LN ;
    icm42688_write_reg(ICM42688_GYRO_CONFIG0, gyro_config) ;
    icm42688_delay(15) ;

    uint8_t accel_config = ICM42688_ACCEL_FS_16G | ICM42688_ODR_4KHZ_LN ;
    icm42688_write_reg(ICM42688_ACCEL_CONFIG0, accel_config) ;
    icm42688_delay(15) ;
}

void icm42688_drdy_exti_handler(void)
{
    icm42688.exti_time_stamp = TIM5_GetStamp() ;
    SPI_Dev_ReadRegs_DMA(&icm42688_hw , icm42688.icm42688_tx_buffer, icm42688.icm42688_rx_rawdata_buf, ICM42688_DATA_LEN) ;
}

void icm42688_get_rawdata(IMU_Raw_t * out_data)
{
    uint32_t seq ;

    if(out_data == NULL) return ; 

    do {
        seq = icm42688_seq_lock ;

        COMPILER_BARRIER() ; 

        *out_data = icm42688.icm42688_raw_data ;

        COMPILER_BARRIER() ; 
    } while ((seq != icm42688_seq_lock) || (seq & 1)) ;
}


void icm42688_register_delay(void (*delay_func)(uint32_t ms))
{
    icm42688.icm42688_delay = delay_func ;
}