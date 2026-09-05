#include "oled/oled_port.h"
#include "iic/ll_iic.h"     
#include <stdbool.h>

#define OLED_I2C_ADDR 0x78

// 包含 13 字节的 I2C 协议头 + 1024 字节的纯净显存
static uint8_t I2C_DMA_Buffer[1037] ; 

// 把去除了 13 个头部字节后的纯净地址交给逻辑层
uint8_t* OLED_HW_GetCanvasPtr(void) {
    return &I2C_DMA_Buffer[13] ; 
}

void OLED_HW_InitBus(void) {
    // 强制固定 DMA 发送的协议头，无论怎么复位，每一帧都能对齐
    
    // 设置列地址 (0x21), 范围 0~127
    I2C_DMA_Buffer[0]  = 0x80 ;  // Co=1, D/C=0 (接下来是单条命令)
    I2C_DMA_Buffer[1]  = 0x21 ;  
    I2C_DMA_Buffer[2]  = 0x80 ;
    I2C_DMA_Buffer[3]  = 0x00 ;  // 起始列 0
    I2C_DMA_Buffer[4]  = 0x80 ;
    I2C_DMA_Buffer[5]  = 0x7F ;  // 终止列 127
    
    // 设置页地址 (0x22), 范围 0~7
    I2C_DMA_Buffer[6]  = 0x80 ;
    I2C_DMA_Buffer[7]  = 0x22 ;
    I2C_DMA_Buffer[8]  = 0x80 ;
    I2C_DMA_Buffer[9]  = 0x00 ;  // 起始页 0
    I2C_DMA_Buffer[10] = 0x80 ;
    I2C_DMA_Buffer[11] = 0x07 ;  // 终止页 7
    
    // 切换为连续数据流模式
    I2C_DMA_Buffer[12] = 0x40 ;  // Co=0, D/C=1 (接下来全是显存数据)
}

void OLED_HW_WriteCmd(uint8_t Command) {
    HW_I2C_WriteCmd_Blocking(OLED_I2C_ADDR, Command) ;
}

void OLED_HW_TriggerUpdate(void) {
    static bool dma_is_running = false ;

    if (dma_is_running) {
        if (!HW_I2C_DMA_Check_And_Stop()) {
            return ; 
        }
        dma_is_running = false ; 
    }
    
    HW_I2C_DMA_Trigger(OLED_I2C_ADDR, I2C_DMA_Buffer, 1037) ;
    dma_is_running = true ;
}