#include "callback_center.h"
#include "dma/ll_dma_spi.h"
#include "dma/ll_dma_iic.h"
#include "exti/ll_exti.h"
#include "spi_device.h"
#include "iic_device.h"
#include "bmm150/bmm150.h"
#include "icm42688/icm42688.h"
#include "bmp390l/bmp390l.h"
#include "sys_hardware_config.h"
#include "app_task/app_task_ahrs.h"

/* ====================================================================
 * DMA TC 适配层 (消除参数差异)
 * 映射关系: DMA 中断 (无参) -> 具体总线 (带参)
 * ==================================================================== */
static void Router_DMA_SPI1_TC(void) 
{
    SPI_Dev_DMA_TC_Handler(SPI1);
}

static void Router_DMA_SPI2_TC(void) 
{
    SPI_Dev_DMA_TC_Handler(SPI2);
}

static void Router_DMA_IIC2_TC(void)
{
    IIC_Dev_DMA_TC_Handler(I2C2) ; 
}

/* ====================================================================
 * 注册中心初始化
 * ==================================================================== */
void CallbackCenter_Init(void)
{
   // 注册 SPI 相关的 DMA 传输完成路由 (下行：DMA中断 -> SPI中间件)
    DMA_SPI_RegisterTCCallback(DEVICE_SPI1, Router_DMA_SPI1_TC);
    DMA_SPI_RegisterTCCallback(DEVICE_SPI2, Router_DMA_SPI2_TC);


    DMA_IIC_RegisterTCCallback(DEVICE_IIC2, Router_DMA_IIC2_TC);
    
    // 注册 SPI 相关的 DMA 触发动作路由 (上行：SPI中间件 -> DMA硬件驱动)
    SPI_Dev_Register_DMA_Trigger(DEVICE_SPI1, DMA_SPI1_TxRx_Trigger);
    SPI_Dev_Register_DMA_Trigger(DEVICE_SPI2, DMA_SPI2_TxRx_Trigger);

    IIC_Dev_Register_DMA_Trigger(DEVICE_IIC2, DMA_IIC2_Rx_Trigger); 
    
    // 注册外部中断路由 (传感器硬件引脚 -> 传感器驱动)
    EXTI_RegisterCallback(BMM150_EXTI_LINE, bmm150_drdy_exti_handler);
    EXTI_RegisterCallback(ICM42688_EXTI_LINE , icm42688_drdy_exti_handler) ; 
    EXTI_RegisterCallback(BMP390L_EXTI_LINE , bmp390l_drdy_exti_handler) ; 

    // 注册任务触发器
    icm42688_ahrs_task_trigger_register(ahrs_task_trigger) ; 
    
}