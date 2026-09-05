#include "spi_device.h"

static SPI_Device_t *s_active_dev[3] = {NULL, NULL, NULL} ;

static void (*s_spi_dma_triggers[3])(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) = {NULL, NULL, NULL};

// 提供给底层的注册接口
void SPI_Dev_Register_DMA_Trigger(uint8_t spi_id, void (*trigger_func)(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)){
    if (spi_id < 3) 
    {
        s_spi_dma_triggers[spi_id] = trigger_func;
    }
}

static uint8_t Get_SPI_Index(SPI_TypeDef *SPIx) 
{
    if (SPIx == SPI1) return 0;
    if (SPIx == SPI2) return 1;
    return 2;
}

static uint8_t SPI_Transmit_Byte(SPI_TypeDef *SPIx, uint8_t data) 
{
    while(!LL_SPI_IsActiveFlag_TXE(SPIx));
    LL_SPI_TransmitData8(SPIx, data);
    
    while(!LL_SPI_IsActiveFlag_RXNE(SPIx));
    return LL_SPI_ReceiveData8(SPIx);
}

void SPI_Dev_WriteReg(SPI_Device_t *dev, uint8_t reg, uint8_t data) 
{
    LL_GPIO_ResetOutputPin(dev->CS_Port, dev->CS_Pin);
    
    SPI_Transmit_Byte(dev->SPIx, reg & 0x7F);
    SPI_Transmit_Byte(dev->SPIx, data);
    
    LL_GPIO_SetOutputPin(dev->CS_Port, dev->CS_Pin);
}

void SPI_Dev_WriteRegs(SPI_Device_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    LL_GPIO_ResetOutputPin(dev->CS_Port, dev->CS_Pin);

    SPI_Transmit_Byte(dev->SPIx, reg & 0x7F);

    for (uint16_t i = 0; i < len; i++) 
    {
        SPI_Transmit_Byte(dev->SPIx, data[i]);
    }
    
    LL_GPIO_SetOutputPin(dev->CS_Port, dev->CS_Pin);
}

uint8_t SPI_Dev_ReadReg(SPI_Device_t *dev, uint8_t reg)
{
    uint8_t rx_data;
    
    LL_GPIO_ResetOutputPin(dev->CS_Port, dev->CS_Pin);
    
    SPI_Transmit_Byte(dev->SPIx, reg | 0x80);
    
    rx_data = SPI_Transmit_Byte(dev->SPIx, 0xFF);
    
    LL_GPIO_SetOutputPin(dev->CS_Port, dev->CS_Pin);
    
    return rx_data;
}

void SPI_Dev_ReadRegs(SPI_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len)
{
    LL_GPIO_ResetOutputPin(dev->CS_Port, dev->CS_Pin);

    SPI_Transmit_Byte(dev->SPIx, reg | 0x80);

    for (uint16_t i = 0; i < len; i++) 
    {
        buffer[i] = SPI_Transmit_Byte(dev->SPIx, 0xFF);
    }
    
    LL_GPIO_SetOutputPin(dev->CS_Port, dev->CS_Pin);
}

void SPI_Dev_ReadRegs_DMA(SPI_Device_t *dev, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) 
{ 
    uint8_t spi_idx = Get_SPI_Index(dev->SPIx);
    
    s_active_dev[spi_idx] = dev; 
    LL_GPIO_ResetOutputPin(dev->CS_Port, dev->CS_Pin);
    
    if (s_spi_dma_triggers[spi_idx] != NULL) 
    {
        s_spi_dma_triggers[spi_idx](tx_buf, rx_buf, len); 
    }
}

void SPI_Dev_DMA_TC_Handler(SPI_TypeDef *SPIx)
{
    // 查出是哪个设备在用这根 SPI
    SPI_Device_t *dev = s_active_dev[Get_SPI_Index(SPIx)];
    
    if (dev != NULL) {
        while(LL_SPI_IsActiveFlag_BSY(SPIx)) ;
        // 框架根据当初保存的句柄，自动拉高对应设备的 CS
        LL_GPIO_SetOutputPin(dev->CS_Port, dev->CS_Pin);

        // 释放总线
        s_active_dev[Get_SPI_Index(SPIx)] = NULL;

        // 呼叫应用层的回调，通知它数据做好了
        if (dev->rx_callback != NULL) 
        {
            dev->rx_callback();
        }
    }
}



