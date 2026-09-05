#include "iic/ll_iic.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_dma.h"

#define I2C_TIMEOUT 500000

// 阻塞发送一条命令
void HW_I2C_WriteCmd_Blocking(uint8_t slave_addr, uint8_t cmd) {
    uint32_t timeout = I2C_TIMEOUT ;
    
    // 等待总线空闲
    while(LL_I2C_IsActiveFlag_BUSY(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }

    // 发送 START
    LL_I2C_GenerateStartCondition(HW_I2C_OLED_INSTANCE) ;
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_SB(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }

    // 发送从机地址 (OLED_I2C_ADDR = 0x78)
    LL_I2C_TransmitData8(HW_I2C_OLED_INSTANCE, slave_addr) ;
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_ADDR(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    LL_I2C_ClearFlag_ADDR(HW_I2C_OLED_INSTANCE) ;

    // 发送写命令标志 (0x00)
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_TXE(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    LL_I2C_TransmitData8(HW_I2C_OLED_INSTANCE, 0x00) ; 
    
    // 发送具体的指令数据 (cmd)
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_TXE(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    LL_I2C_TransmitData8(HW_I2C_OLED_INSTANCE, cmd) ;

    // 等待移位寄存器清空并发送 STOP
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_BTF(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    LL_I2C_GenerateStopCondition(HW_I2C_OLED_INSTANCE) ;
    
    // 强制等待总线完全释放
    timeout = I2C_TIMEOUT ;
    while(LL_I2C_IsActiveFlag_BUSY(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
}

// 2. 检查 DMA 是否完成并发送 Stop (收尾动作)
bool HW_I2C_DMA_Check_And_Stop(void) {
    // 如果 DMA 还没搬完，返回 false
    if (!LL_DMA_IsActiveFlag_TC6(HW_DMA_OLED_I2C_TX_INSTANCE)) {
        return false ; 
    }

    // DMA 搬完，执行收尾
    LL_DMA_ClearFlag_TC6(HW_DMA_OLED_I2C_TX_INSTANCE) ;
    uint32_t timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_BTF(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) break ; }
    
    LL_I2C_GenerateStopCondition(HW_I2C_OLED_INSTANCE) ;
    timeout = I2C_TIMEOUT ;
    while(LL_I2C_IsActiveFlag_BUSY(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) break ; }

    LL_I2C_DisableDMAReq_TX(HW_I2C_OLED_INSTANCE) ;
    LL_DMA_DisableStream(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM) ;
    
    return true ; // 成功收尾释放
}

//  配置目标地址并启动 DMA 
void HW_I2C_DMA_Trigger(uint8_t slave_addr, uint8_t *data, uint16_t len) {
    uint32_t timeout = I2C_TIMEOUT ;
    while(LL_I2C_IsActiveFlag_BUSY(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    
    LL_I2C_GenerateStartCondition(HW_I2C_OLED_INSTANCE) ;
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_SB(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    
    LL_I2C_TransmitData8(HW_I2C_OLED_INSTANCE, slave_addr) ; 
    timeout = I2C_TIMEOUT ;
    while(!LL_I2C_IsActiveFlag_ADDR(HW_I2C_OLED_INSTANCE)) { if(--timeout == 0) return ; }
    LL_I2C_ClearFlag_ADDR(HW_I2C_OLED_INSTANCE) ;

    // 重新装载 DMA
    LL_DMA_SetMemoryAddress(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, (uint32_t)data) ;
    LL_DMA_SetDataLength(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM, len) ;
    
    LL_I2C_EnableDMAReq_TX(HW_I2C_OLED_INSTANCE) ;
    LL_DMA_EnableStream(HW_DMA_OLED_I2C_TX_INSTANCE, HW_DMA_OLED_I2C_TX_STREAM) ;
}

void OLED_IIC_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1) ;

    LL_I2C_Disable(HW_I2C_OLED_INSTANCE) ; 
    LL_I2C_InitTypeDef I2C_InitStruct = {0} ;
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C ;
    I2C_InitStruct.ClockSpeed = HW_I2C_OLED_SPEED ; 
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2 ;
    I2C_InitStruct.OwnAddress1 = 0 ;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK ;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT ;
    LL_I2C_Init(HW_I2C_OLED_INSTANCE, &I2C_InitStruct) ;
    LL_I2C_Enable(HW_I2C_OLED_INSTANCE) ;
}