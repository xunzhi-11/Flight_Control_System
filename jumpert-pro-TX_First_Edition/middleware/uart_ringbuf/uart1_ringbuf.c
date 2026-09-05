#include "uart/ll_uart.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "string.h"


#define CRSF_Frame_Length  26
#define Single_Buffer_FrameNum 5
#define UART1_RX_BUFFER_SIZE Single_Buffer_FrameNum * CRSF_Frame_Length


typedef struct 
{
    uint8_t Buffer[UART1_RX_BUFFER_SIZE] ; 
    uint8_t Read_Index ; 
} RX_Buffer ;

typedef struct 
{
    uint8_t Buffer[UART1_RX_BUFFER_SIZE] ; 
    uint8_t tx_isbusy_lock ; 
} TX_Buffer ;


typedef struct 
{
    RX_Buffer UART1_Read_Buffer ; 
    TX_Buffer UART1_Write_Buffer ; 
}UART1_Buffer ;

UART1_Buffer uart1_buffer = {0} ; 


uint8_t* UART1_Get_DMA_ReadBuffer_Addr(void)
{
    return uart1_buffer.UART1_Read_Buffer.Buffer ; 
}

uint8_t* UART1_Get_DMA_WriteBuffer_Addr(void)   
{
    return uart1_buffer.UART1_Write_Buffer.Buffer ; 
}

uint16_t UART1_Get_DMA_Buffer_Size(void)
{
    return UART1_RX_BUFFER_SIZE ; 
}

uint8_t UART1_Update_Byte(uint8_t * out_put)
{
    uint16_t write_index  = UART1_RX_BUFFER_SIZE 
                            - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5) ;
        
    if(write_index == UART1_RX_BUFFER_SIZE)
    {
        write_index = 0 ; 
    }

    if (uart1_buffer.UART1_Read_Buffer.Read_Index == write_index)
    {
        return 0 ;
    }

    *out_put = uart1_buffer.UART1_Read_Buffer.Buffer[uart1_buffer.UART1_Read_Buffer.Read_Index] ; 

    uart1_buffer.UART1_Read_Buffer.Read_Index ++ ; 

    if (uart1_buffer.UART1_Read_Buffer.Read_Index >= UART1_RX_BUFFER_SIZE) 
    {
        uart1_buffer.UART1_Read_Buffer.Read_Index = 0 ;
    }
    
    return 1 ;
}   

static void UART1_TX_Lockup(void)
{
    uart1_buffer.UART1_Write_Buffer.tx_isbusy_lock = 1 ; 
}


void UART1_TX_Lockoff(void)
{
    uart1_buffer.UART1_Write_Buffer.tx_isbusy_lock = 0 ; 
}


uint8_t UART1_Transmit_DMA_Async(const uint8_t *data, uint16_t len)
{
    if (len > 64 || uart1_buffer.UART1_Write_Buffer.tx_isbusy_lock == 1)
    {
        return 0 ;
    } 
    
    UART1_TX_Lockup() ; 
    
    memcpy(uart1_buffer.UART1_Write_Buffer.Buffer, data, len) ;
    
    LL_USART_DisableDirectionRx(USART1) ;
    LL_USART_EnableDirectionTx(USART1) ;
    LL_USART_ClearFlag_TC(USART1) ;
    
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7) ;
    
    uint32_t timeout = 10000 ; 
    while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7))
    {
        timeout-- ;
        if(timeout == 0) 
        {
            break ; 
        }
    }
    
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)uart1_buffer.UART1_Write_Buffer.Buffer) ;

    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, len) ;

    LL_DMA_ClearFlag_TC7(DMA2) ;
    LL_DMA_ClearFlag_HT7(DMA2) ;
    LL_DMA_ClearFlag_TE7(DMA2) ;
    LL_DMA_ClearFlag_DME7(DMA2) ;
    LL_DMA_ClearFlag_FE7(DMA2) ;

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7) ;
    return 1 ;
}