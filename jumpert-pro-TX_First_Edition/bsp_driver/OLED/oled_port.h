// oled_port.h
#ifndef __OLED_PORT_H
#define __OLED_PORT_H

#include <stdint.h>

// 初始化底层总线 (I2C+DMA等)
void OLED_HW_InitBus(void) ;

// 阻塞发送一个指令 (用于初始化屏幕)
void OLED_HW_WriteCmd(uint8_t cmd) ;

// 触发一帧底层 DMA 发送 (非阻塞)
void OLED_HW_TriggerUpdate(void) ;

// 向底层索要显存指针
uint8_t* OLED_HW_GetCanvasPtr(void) ;

#endif