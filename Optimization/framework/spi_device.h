#ifndef __SPI_DEVICE_H
#define __SPI_DEVICE_H

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"

// 定义一个“SPI从设备”类
typedef struct {
    SPI_TypeDef  *SPIx;       // 绑定的 SPI 总线 
    GPIO_TypeDef *CS_Port;    // 绑定的 CS 引脚端口 
    uint32_t      CS_Pin;     // 绑定的 CS 引脚编号 
    void          (*dma_trigger_func)(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);
    void          (*rx_callback)(void);
} SPI_Device_t;

void SPI_Dev_Register_DMA_Trigger(uint8_t spi_id, void (*trigger_func)(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)); 
void SPI_Dev_WriteReg(SPI_Device_t *dev, uint8_t reg, uint8_t data) ; 
void SPI_Dev_WriteRegs(SPI_Device_t *dev, uint8_t reg, uint8_t *data, uint16_t len) ; 
uint8_t SPI_Dev_ReadReg(SPI_Device_t *dev, uint8_t reg) ; 
void SPI_Dev_ReadRegs(SPI_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len) ; 
void SPI_Dev_ReadRegs_DMA(SPI_Device_t *dev, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)  ;
void SPI_Dev_DMA_TC_Handler(SPI_TypeDef *SPIx) ;

#endif
