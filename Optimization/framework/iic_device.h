#ifndef __IIC_DEVICE_H
#define __IIC_DEVICE_H

#include "stm32f4xx_ll_i2c.h"

// 定义一个“IIC从设备”类
typedef struct {
    I2C_TypeDef  *I2Cx;             // 绑定的 I2C 总线 
    uint8_t       DevAddress;       // 从设备 I2C 地址 (包含读写位偏移，8-bit 地址)
    
    void          (*dma_trigger_func)(uint8_t *buf, uint16_t len); // 底层 DMA 触发接口
    void          (*rx_callback)(void);                            // DMA 接收完成回调应用层
} IIC_Device_t;

// 接口声明
void IIC_Dev_Register_DMA_Trigger(uint8_t iic_id, void (*trigger_func)(uint8_t*, uint16_t));
uint8_t IIC_Dev_Is_Busy(uint8_t dev_id) ; 


void IIC_Dev_WriteReg(IIC_Device_t *dev, uint8_t reg, uint8_t data);
void IIC_Dev_WriteRegs(IIC_Device_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

uint8_t IIC_Dev_ReadReg(IIC_Device_t *dev, uint8_t reg);
void IIC_Dev_ReadRegs(IIC_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len);

void IIC_Dev_ReadRegs_DMA(IIC_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len);
void IIC_Dev_DMA_TC_Handler(I2C_TypeDef *I2Cx);

#endif