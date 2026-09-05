#include "OLED/OLED.h"
#include "OLED/OLED_Font.h"
#include "oled_port.h"     
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// 逻辑层不再定义数组，直接找底层索要指针，并强转为二维数组
#define OLED_Buffer ((uint8_t (*)[128])OLED_HW_GetCanvasPtr())

static uint8_t current_page = 0 ;
static uint8_t current_col = 0 ;

/**
  * @brief  纯内存操作：设置逻辑光标
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    current_page = Y ;
    current_col = X ;
}

/**
  * @brief  纯内存操作：写入显存缓冲区
  */
void OLED_WriteData(uint8_t Data)
{
    if (current_page < 8 && current_col < 128) {
        OLED_Buffer[current_page][current_col] = Data ;
        current_col++ ;
    }
}

void OLED_WriteData_ToBuffer(uint8_t page, uint8_t col, uint8_t data)
{
    if (page < 8 && col < 128) {
        OLED_Buffer[page][col] = data ;
    }
}

/**
  * @brief  把触发任务甩给底层接口
  */
void OLED_Update(void)
{
    OLED_HW_TriggerUpdate() ; 
}

/**
  * @brief  纯内存操作：清空缓冲区
  */
void OLED_Clear(void)
{  
    for (uint8_t j = 0 ; j < 8 ; j++) {
        for (uint8_t i = 0 ; i < 128 ; i++) {
            OLED_Buffer[j][i] = 0x00 ;
        }
    }
}

/**
  * @brief  纯内存操作：画点
  */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode)
{
    if (x >= 128 || y >= 64) return ;
    
    uint8_t page = y / 8 ;     
    uint8_t bit_pos = y % 8 ;  
    
    if (mode) {
        OLED_Buffer[page][x] |= (1 << bit_pos) ;  
    } else {
        OLED_Buffer[page][x] &= ~(1 << bit_pos) ; 
    }
}

void OLED_Init(void)
{
    uint32_t i, j ;
    for (i = 0 ; i < 1000 ; i++) { for (j = 0 ; j < 1000 ; j++) ; } // 上电延时

    OLED_HW_InitBus() ;            
    OLED_HW_WriteCmd(0xAE) ;    // 关闭显示
    OLED_HW_WriteCmd(0xD5) ;    // 设置显示时钟分频比/振荡器频率
    OLED_HW_WriteCmd(0x80) ;
    OLED_HW_WriteCmd(0xA8) ;    // 设置多路复用率
    OLED_HW_WriteCmd(0x3F) ;
    OLED_HW_WriteCmd(0xD3) ;    // 设置显示偏移
    OLED_HW_WriteCmd(0x00) ;
    OLED_HW_WriteCmd(0x40) ;    // 设置显示开始行
    
    OLED_HW_WriteCmd(0x20) ;    // 水平寻址模式 (支持全屏连续写入)
    OLED_HW_WriteCmd(0x00) ;    

    OLED_HW_WriteCmd(0xA1) ;    // 设置左右方向
    OLED_HW_WriteCmd(0xC8) ;    // 设置上下方向
    OLED_HW_WriteCmd(0xDA) ;    // 设置COM引脚硬件配置
    OLED_HW_WriteCmd(0x12) ;
    OLED_HW_WriteCmd(0x81) ;    // 设置对比度控制
    OLED_HW_WriteCmd(0xCF) ;
    OLED_HW_WriteCmd(0xD9) ;    // 设置预充电周期
    OLED_HW_WriteCmd(0xF1) ;
    OLED_HW_WriteCmd(0xDB) ;    // 设置VCOMH取消选择级别
    OLED_HW_WriteCmd(0x30) ;
    OLED_HW_WriteCmd(0xA4) ;    // 设置整个显示打开/关闭
    OLED_HW_WriteCmd(0xA6) ;    // 设置正常/倒转显示
    OLED_HW_WriteCmd(0x8D) ;    // 设置充电泵
    OLED_HW_WriteCmd(0x14) ;
    OLED_HW_WriteCmd(0xAF) ;    // 开启显示
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{       
    uint8_t i ;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8) ;       
    for (i = 0 ; i < 8 ; i++) { OLED_WriteData(OLED_F8x16[Char - ' '][i]) ; }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8) ;   
    for (i = 0 ; i < 8 ; i++) { OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]) ; }
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i ;
    for (i = 0 ; String[i] != '\0' ; i++) { OLED_ShowChar(Line, Column + i, String[i]) ; }
}

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1 ;
    while (Y--) { Result *= X ; }
    return Result ;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i ;
    for (i = 0 ; i < Length ; i++) { OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0') ; }
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i ;
    uint32_t Number1 ;
    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+') ;
        Number1 = Number ;
    } else {
        OLED_ShowChar(Line, Column, '-') ;
        Number1 = -Number ;
    }
    for (i = 0 ; i < Length ; i++) { OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0') ; }
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber ;
    for (i = 0 ; i < Length ; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16 ;
        if (SingleNumber < 10) { OLED_ShowChar(Line, Column + i, SingleNumber + '0') ; }
        else { OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A') ; }
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i ;
    for (i = 0 ; i < Length ; i++) { OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0') ; }
}

void OLED_ShowCN(uint8_t Line, uint8_t Column, uint8_t Num, uint8_t mode)  
{      
    uint8_t i ;
    uint8_t page_start = (Line - 1) * 2 ;   
    uint8_t col_start = (Column - 1) * 16 ; 

    OLED_SetCursor(page_start, col_start) ;
    for (i = 0 ; i < 16 ; i++) { OLED_WriteData(mode ? OLED_F10x16[Num][i] : ~OLED_F10x16[Num][i]) ; }
    OLED_SetCursor(page_start + 1, col_start) ;
    for (i = 16 ; i < 32 ; i++) { OLED_WriteData(mode ? OLED_F10x16[Num][i] : ~OLED_F10x16[Num][i]) ; }
}

void OLED_ShowBMP(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bmp, uint8_t mode) 
{
    uint8_t page_end = y + height / 8 ;
    uint8_t col_end = x + width ;
    uint16_t index = 0 ;
    
    for (uint8_t page = y ; page < page_end ; page++) {
        OLED_SetCursor(page, x) ;
        for (uint8_t col = x ; col < col_end ; col++) {
            uint8_t data = bmp[index++] ;
            OLED_WriteData(mode ? data : ~data) ;
        }
    }
}

#define OLED_MAX_LINES      4       
#define OLED_MAX_COLUMNS    16      
#define FLOAT_BUFFER_SIZE   20      
#define FLOAT_MAX_VALUE     999999.9999f   
#define FLOAT_MIN_VALUE     (-999999.9999f) 
#define FLOAT_MAX_DECIMALS  4       

static const uint32_t s_pow10_table[8] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 } ;

static uint8_t uint32_to_str(uint32_t value, char *buf, uint8_t buf_size)
{
    if (buf == NULL || buf_size < 2) return 0 ;
    char temp[12] ;  
    uint8_t len = 0 ;
    if (value == 0) { buf[0] = '0' ; buf[1] = '\0' ; return 1 ; }
    while (value > 0 && len < 10) { temp[len++] = (value % 10) + '0' ; value /= 10 ; }
    if (len >= buf_size) { buf[0] = '\0' ; return 0 ; }
    for (uint8_t i = 0 ; i < len ; i++) { buf[i] = temp[len - 1 - i] ; }
    buf[len] = '\0' ;
    return len ;
}

void OLED_ShowFloat(uint8_t Line, uint8_t Column, float num, uint8_t decimal_len)
{
    char buffer[FLOAT_BUFFER_SIZE] ;
    uint8_t index = 0 ;
    if (Line < 1 || Line > OLED_MAX_LINES || Column < 1 || Column > OLED_MAX_COLUMNS) return ;  
    if (decimal_len > FLOAT_MAX_DECIMALS) decimal_len = FLOAT_MAX_DECIMALS ;  
    if (isnan(num)) { OLED_ShowString(Line, Column, "NaN  ") ; return ; }
    if (isinf(num) && num > 0) { OLED_ShowString(Line, Column, "+Inf ") ; return ; }
    if (isinf(num) && num < 0) { OLED_ShowString(Line, Column, "-Inf ") ; return ; }
    if (num > FLOAT_MAX_VALUE) { OLED_ShowString(Line, Column, "+OVF ") ; return ; }
    if (num < FLOAT_MIN_VALUE) { OLED_ShowString(Line, Column, "-OVF ") ; return ; }
    
    bool is_negative = false ;
    if (num < 0) { is_negative = true ; num = -num ; buffer[index++] = '-' ; }
    float rounding = 0.5f / (float)s_pow10_table[decimal_len] ;
    num += rounding ;
    if (num > (float)(FLOAT_MAX_VALUE + 1)) { OLED_ShowString(Line, Column, is_negative ? "-OVF " : "+OVF ") ; return ; }
    
    uint32_t integer_part = (uint32_t)num ;
    float fractional = num - (float)integer_part ;
    
    char int_buf[12] ;
    uint8_t int_len = uint32_to_str(integer_part, int_buf, sizeof(int_buf)) ;
    if (index + int_len >= FLOAT_BUFFER_SIZE - decimal_len - 2) { OLED_ShowString(Line, Column, "ERR  ") ; return ; }
    for (uint8_t i = 0 ; i < int_len ; i++) { buffer[index++] = int_buf[i] ; }
    
    if (decimal_len > 0) {
        buffer[index++] = '.' ;
        uint32_t frac_int = (uint32_t)(fractional * (float)s_pow10_table[decimal_len]) ;
        uint32_t max_frac = s_pow10_table[decimal_len] - 1 ;
        if (frac_int > max_frac) frac_int = max_frac ;
        for (int8_t i = decimal_len - 1 ; i >= 0 ; i--) { buffer[index + i] = (frac_int % 10) + '0' ; frac_int /= 10 ; }
        index += decimal_len ;
    }
    buffer[index] = '\0' ;
    if (index >= FLOAT_BUFFER_SIZE) { OLED_ShowString(Line, Column, "BUF! ") ; return ; }
    uint8_t available_width = OLED_MAX_COLUMNS - Column + 1 ;
    if (index > available_width) { buffer[available_width - 1] = '>' ; buffer[available_width] = '\0' ; }
    OLED_ShowString(Line, Column, buffer) ;
}