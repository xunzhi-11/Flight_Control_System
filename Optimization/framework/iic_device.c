#include "iic_device.h"
#include <stddef.h>

static IIC_Device_t *s_active_device[3] = {NULL, NULL, NULL}; 
static void (*s_iic_dma_triggers[3])(uint8_t *rx_buf, uint16_t len) = {NULL, NULL, NULL};

// 提供给底层的注册接口
void IIC_Dev_Register_DMA_Trigger(uint8_t iic_id, void (*trigger_func)(uint8_t*, uint16_t))
{
    if (iic_id < 3) 
    {
        s_iic_dma_triggers[iic_id] = trigger_func;
    }
}

__attribute__((always_inline)) inline uint8_t IIC_Dev_Is_Busy(uint8_t dev_id)
{
    return (s_active_device[dev_id] ? 1 : 0) ; 
}

static uint8_t Get_IIC_Index(I2C_TypeDef *I2Cx) 
{
    if (I2Cx == I2C1) return 0;
    if (I2Cx == I2C2) return 1;
    return 2; 
}

static uint8_t IIC_Start_And_SendAddr(I2C_TypeDef *I2Cx, uint8_t addr_8bit)
{
    uint32_t timeout;
    
    // 清除所有可能的错误标志
    LL_I2C_ClearFlag_BERR(I2Cx);
    LL_I2C_ClearFlag_AF(I2Cx);
    LL_I2C_ClearFlag_OVR(I2Cx);
    LL_I2C_ClearFlag_ARLO(I2Cx);

    LL_I2C_GenerateStartCondition(I2Cx);
    
    timeout = 10000; 
    while(!LL_I2C_IsActiveFlag_SB(I2Cx))
    {
        if(--timeout == 0) return 1; 
    }
    
    // 发送从机地址
    LL_I2C_TransmitData8(I2Cx, addr_8bit);
    
    // 等待 ADDR 标志位
    timeout = 10000;
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx))
    {
        //  NACK，应答失败
        if (LL_I2C_IsActiveFlag_AF(I2Cx) || --timeout == 0) return 1; 
    }
    
    LL_I2C_ClearFlag_ADDR(I2Cx);
    
    return 0; 
}

void IIC_Dev_WriteReg(IIC_Device_t *dev, uint8_t reg, uint8_t data) 
{
    IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress & 0xFE); // 寻址 + 写方向
    
    // 发送寄存器地址
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, reg);
    
    // 发送数据
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, data);
    
    // 等待字节发送完成 (BTF) 后产生 STOP 信号
    while(!LL_I2C_IsActiveFlag_BTF(dev->I2Cx));
    LL_I2C_GenerateStopCondition(dev->I2Cx);
}

void IIC_Dev_WriteRegs(IIC_Device_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress & 0xFE); 
    
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, reg);
    
    for (uint16_t i = 0; i < len; i++) 
    {
        while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
        LL_I2C_TransmitData8(dev->I2Cx, data[i]);
    }
    
    while(!LL_I2C_IsActiveFlag_BTF(dev->I2Cx));
    LL_I2C_GenerateStopCondition(dev->I2Cx);
}

uint8_t IIC_Dev_ReadReg(IIC_Device_t *dev, uint8_t reg)
{
    uint8_t rx_data;
    
    // 写阶段 (指定寄存器)
    IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress & 0xFE);
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, reg);
    while(!LL_I2C_IsActiveFlag_BTF(dev->I2Cx)); // 确保写操作完成
    
    // 读阶段 (Restart)
    LL_I2C_GenerateStartCondition(dev->I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(dev->I2Cx));
    
    LL_I2C_TransmitData8(dev->I2Cx, dev->DevAddress | 0x01); // 寻址 + 读方向
    while(!LL_I2C_IsActiveFlag_ADDR(dev->I2Cx));
    
    // STM32F4 读取单字节特殊规则：先禁用 ACK，清 ADDR，产生 STOP，再读数据
    LL_I2C_AcknowledgeNextData(dev->I2Cx, LL_I2C_NACK);
    LL_I2C_ClearFlag_ADDR(dev->I2Cx);
    LL_I2C_GenerateStopCondition(dev->I2Cx);
    
    while(!LL_I2C_IsActiveFlag_RXNE(dev->I2Cx));
    rx_data = LL_I2C_ReceiveData8(dev->I2Cx);
    
    // 恢复默认 ACK 状态
    LL_I2C_AcknowledgeNextData(dev->I2Cx, LL_I2C_ACK);
    
    return rx_data;
}

void IIC_Dev_ReadRegs(IIC_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len)
{
    if(len == 0) return;
    
    // 写阶段 (指定寄存器)
    IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress & 0xFE);
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, reg);
    while(!LL_I2C_IsActiveFlag_BTF(dev->I2Cx));
    
    // 读阶段 (Restart)
    LL_I2C_GenerateStartCondition(dev->I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(dev->I2Cx));
    LL_I2C_TransmitData8(dev->I2Cx, dev->DevAddress | 0x01);
    while(!LL_I2C_IsActiveFlag_ADDR(dev->I2Cx));
    
    if (len == 1) 
    {
        LL_I2C_AcknowledgeNextData(dev->I2Cx, LL_I2C_NACK);
        LL_I2C_ClearFlag_ADDR(dev->I2Cx);
        LL_I2C_GenerateStopCondition(dev->I2Cx);
    } 
    else 
    {
        LL_I2C_ClearFlag_ADDR(dev->I2Cx);
    }
    
    for (uint16_t i = 0; i < len; i++) 
    {
        if (i == len - 1 && len > 1) // 最后一个字节前设置 NACK 和 STOP
        {
            LL_I2C_AcknowledgeNextData(dev->I2Cx, LL_I2C_NACK);
            LL_I2C_GenerateStopCondition(dev->I2Cx);
        }
        
        while(!LL_I2C_IsActiveFlag_RXNE(dev->I2Cx));
        buffer[i] = LL_I2C_ReceiveData8(dev->I2Cx);
    }
    
    LL_I2C_AcknowledgeNextData(dev->I2Cx, LL_I2C_ACK); // 恢复
}

void IIC_Dev_ReadRegs_DMA(IIC_Device_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len) 
{
    uint8_t iic_idx = Get_IIC_Index(dev->I2Cx);
    
    // 记录当前抢占总线的设备 
    s_active_device[iic_idx] = dev; 
    
    if (IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress & 0xFE) != 0)
    {
        // 让硬件发 STOP 释放总线
        LL_I2C_GenerateStopCondition(dev->I2Cx); 
        // 手动释放总线
        s_active_device[iic_idx] = NULL; 
        return; 
    }
    
    uint32_t timeout = 10000;
    while(!LL_I2C_IsActiveFlag_TXE(dev->I2Cx)) { if(--timeout == 0) goto iic_error; }
    
    LL_I2C_TransmitData8(dev->I2Cx, reg);
    
    timeout = 10000;
    while(!LL_I2C_IsActiveFlag_BTF(dev->I2Cx)) { if(--timeout == 0) goto iic_error; }
    
    // 发送 Restart，超时检测
    if (IIC_Start_And_SendAddr(dev->I2Cx, dev->DevAddress | 0x01) != 0) goto iic_error;
   
    LL_I2C_EnableDMAReq_RX(dev->I2Cx);
    LL_I2C_EnableLastDMA(dev->I2Cx); 

    if(s_iic_dma_triggers[iic_idx]) 
    {
        s_iic_dma_triggers[iic_idx](buffer, len);
    }
    
    LL_I2C_ClearFlag_ADDR(dev->I2Cx);
    return; // 正常流程结束

// 统一的异常处理出口
iic_error:
    LL_I2C_GenerateStopCondition(dev->I2Cx);
    s_active_device[iic_idx] = NULL; // 超时释放总线
    return;
}


void IIC_Dev_DMA_TC_Handler(I2C_TypeDef *I2Cx)
{
    uint8_t idx = Get_IIC_Index(I2Cx);
    IIC_Device_t *dev = s_active_device[idx];

        // DMA 搬运完，由框架产生 STOP 结束通信
        LL_I2C_GenerateStopCondition(dev->I2Cx);
        
        LL_I2C_DisableDMAReq_RX(dev->I2Cx);
        LL_I2C_DisableLastDMA(dev->I2Cx);

        // 释放总线
        s_active_device[idx] = NULL;

        // 呼叫应用层回调
        dev->rx_callback();
}
