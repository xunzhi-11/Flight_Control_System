#include "nvic/ll_nvic.h"
#include "stm32f4xx_ll_exti.h"
#include "sys_hardware_config.h"

void nvic_dma_to_spi1_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_SPI1_RX, HW_DMA_TO_SPI1_IRQ_PRIORITY) ; 
    NVIC_EnableIRQ(HW_IRQ_DMA_SPI1_RX) ;
}

void nvic_dma_to_spi2_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_SPI2_RX, HW_DMA_TO_SPI2_IRQ_PRIORITY) ; 
    NVIC_EnableIRQ(HW_IRQ_DMA_SPI2_RX) ;
}

void nvic_dma_to_iic2_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_IIC2_RX , HW_DMA_TO_IIC2_IRQ_PRIORITY) ; 
    NVIC_EnableIRQ(HW_IRQ_DMA_IIC2_RX) ; 
}

void nvic_dma_to_uart2_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_UART2_TX, HW_DMA_UART2_TX_IRQ_PRIORITY); 
    NVIC_EnableIRQ(HW_IRQ_DMA_UART2_TX); 
}

void nvic_dma_to_tim1_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_DSHOT , HW_DMA_DSHOT_IRQ_PRIORITY) ; 
    NVIC_EnableIRQ(HW_IRQ_DMA_DSHOT) ;
}

void nvic_dma_to_sdio_config(void)
{
    NVIC_SetPriority(HW_IRQ_DMA_SDIO , HW_DMA_SDIO_IRQ_PRIORITY);
    NVIC_EnableIRQ(HW_IRQ_DMA_SDIO);
}

void nvic_ahrs_task_config(void)
{
    NVIC_SetPriority(HW_IRQ_AHRS_TASK, HW_AHRS_TASK_PRIORITY);
    NVIC_EnableIRQ(HW_IRQ_AHRS_TASK);
}