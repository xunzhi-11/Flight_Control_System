#include "uart/ll_uart.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "sys_hardware_config.h"
#include "vib_log/vib_log_config.h"
#include "pid_trace/pid_trace_config.h"
#if PIDTRACE_ENABLE
#include "pid_trace/pid_trace.h"
#endif

#define CRSF_Frame_Length  26
#define Single_Buffer_FrameNum 30
#define UART2_RX_BUFFER_SIZE Single_Buffer_FrameNum * CRSF_Frame_Length

#define UART3_RX_BUFFER_SIZE 2048U

#ifdef __GNUC__
    int _write(int file, char *ptr, int len)
    {
#if VIBLOG_ENABLE
        (void)file;
        (void)ptr;
        (void)len;
        return len;
#elif PIDTRACE_ENABLE
        if (PidTrace_IsDumping()) {
            (void)file;
            (void)ptr;
            (void)len;
            return len;
        }
        for (int i = 0; i < len; i++) {
            while (!LL_USART_IsActiveFlag_TXE(USART1)) {
            }
            LL_USART_TransmitData8(USART1, (uint8_t)ptr[i]);
        }
        return len;
#else
        for(int i = 0; i < len; i++)
        {
            while(!LL_USART_IsActiveFlag_TXE(USART1));
            LL_USART_TransmitData8(USART1, (uint8_t)ptr[i]);
        }
        return len;
#endif
    }
#endif

typedef struct 
{
    uint8_t Buffer[UART2_RX_BUFFER_SIZE] ; 
    uint8_t Read_Index ; 
} RX_Buffer;

typedef struct 
{
    uint8_t Buffer[UART2_RX_BUFFER_SIZE] ; 
    uint8_t tx_isbusy_lock ; 
} TX_Buffer;

typedef struct 
{
    RX_Buffer UART2_Read_Buffer ; 
    TX_Buffer UART2_Write_Buffer ; 
}UART2_Buffer;

UART2_Buffer uart2_buffer = {0} ; 

typedef struct
{
    uint8_t Buffer[UART3_RX_BUFFER_SIZE];
    uint16_t Read_Index;
} UART3_RX_Buffer;

typedef struct
{
    UART3_RX_Buffer UART3_Read_Buffer;
} UART3_Buffer;

UART3_Buffer uart3_buffer = {0};

static uint16_t UART3_Get_DMA_Write_Index(void)
{
    uint16_t write_index = UART3_RX_BUFFER_SIZE
                           - LL_DMA_GetDataLength(HW_DMA_UART3_RX_INSTANCE, HW_DMA_UART3_RX_STREAM);

    if (write_index >= UART3_RX_BUFFER_SIZE)
    {
        write_index = 0;
    }

    return write_index;
}

uint8_t* UART2_Get_DMA_ReadBuffer_Addr(void)
{
    return uart2_buffer.UART2_Read_Buffer.Buffer ; 
}

uint8_t* UART2_Get_DMA_WriteBuffer_Addr(void)   
{
    return uart2_buffer.UART2_Write_Buffer.Buffer ; 
}

uint16_t UART2_Get_DMA_Buffer_Size(void)
{
    return UART2_RX_BUFFER_SIZE ; 
}

uint8_t UART2_Update_Byte(uint8_t * out_put)
{
    // USART2 的 RX 映射在 DMA1_STREAM_5
    uint16_t write_index  = UART2_RX_BUFFER_SIZE 
                            - LL_DMA_GetDataLength(HW_DMA_UART2_RX_INSTANCE, HW_DMA_UART2_RX_STREAM);
        
    if(write_index == UART2_RX_BUFFER_SIZE)
    {
        write_index = 0 ; 
    }

    if (uart2_buffer.UART2_Read_Buffer.Read_Index == write_index)
    {
        return 0;
    }

    *out_put = uart2_buffer.UART2_Read_Buffer.Buffer[uart2_buffer.UART2_Read_Buffer.Read_Index] ; 

    uart2_buffer.UART2_Read_Buffer.Read_Index ++ ; 

    if (uart2_buffer.UART2_Read_Buffer.Read_Index >= UART2_RX_BUFFER_SIZE) 
    {
        uart2_buffer.UART2_Read_Buffer.Read_Index = 0;
    }
    
    return 1;
}

uint8_t* UART3_Get_DMA_ReadBuffer_Addr(void)
{
    return uart3_buffer.UART3_Read_Buffer.Buffer;
}

uint16_t UART3_Get_DMA_Buffer_Size(void)
{
    return UART3_RX_BUFFER_SIZE;
}

uint16_t UART3_Get_Available_Byte_Count(void)
{
    uint16_t write_index = UART3_Get_DMA_Write_Index();
    uint16_t read_index = uart3_buffer.UART3_Read_Buffer.Read_Index;

    if (write_index >= read_index)
    {
        return write_index - read_index;
    }

    return UART3_RX_BUFFER_SIZE - read_index + write_index;
}

uint8_t UART3_Update_Byte(uint8_t *out_put)
{
    uint16_t write_index = UART3_Get_DMA_Write_Index();

    if (uart3_buffer.UART3_Read_Buffer.Read_Index == write_index)
    {
        return 0;
    }

    *out_put = uart3_buffer.UART3_Read_Buffer.Buffer[uart3_buffer.UART3_Read_Buffer.Read_Index];
    uart3_buffer.UART3_Read_Buffer.Read_Index++;

    if (uart3_buffer.UART3_Read_Buffer.Read_Index >= UART3_RX_BUFFER_SIZE)
    {
        uart3_buffer.UART3_Read_Buffer.Read_Index = 0;
    }

    return 1;
}

uint16_t UART3_Read_Bytes(uint8_t *dst, uint16_t max_len)
{
    uint16_t copied = 0;
    uint8_t byte;

    if (dst == NULL || max_len == 0)
    {
        return 0;
    }

    while (copied < max_len && UART3_Update_Byte(&byte))
    {
        dst[copied++] = byte;
    }

    return copied;
}

static void UART2_TX_Lockup(void)
{
    uart2_buffer.UART2_Write_Buffer.tx_isbusy_lock = 1 ; 
}

void UART2_TX_Lockoff(void)
{
    uart2_buffer.UART2_Write_Buffer.tx_isbusy_lock = 0 ; 
}

uint8_t UART2_Transmit_DMA_Async(const uint8_t *data, uint16_t len)
{
    if (len > 64 || uart2_buffer.UART2_Write_Buffer.tx_isbusy_lock == 1)
    {
        return 0 ;
    } 
    
    UART2_TX_Lockup() ; 
    
    memcpy(uart2_buffer.UART2_Write_Buffer.Buffer, data, len);
    
    // USART2 的 TX 映射在 DMA1_STREAM_6
    LL_DMA_DisableStream(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM) ;
    while(LL_DMA_IsEnabledStream(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM)); 
    
    LL_DMA_SetMemoryAddress(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM, (uint32_t)uart2_buffer.UART2_Write_Buffer.Buffer) ;

    LL_DMA_SetDataLength(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM, len) ;

    // 清除 DMA1_STREAM_6 的相关标志位
    LL_DMA_ClearFlag_TC6(HW_DMA_UART2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_HT6(HW_DMA_UART2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_TE6(HW_DMA_UART2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_DME6(HW_DMA_UART2_TX_INSTANCE) ;
    LL_DMA_ClearFlag_FE6(HW_DMA_UART2_TX_INSTANCE) ;

    LL_DMA_EnableStream(HW_DMA_UART2_TX_INSTANCE, HW_DMA_UART2_TX_STREAM) ;
    return 1 ;
}

void UART_Config_Init(void)
{

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1) ;

    LL_USART_Disable(HW_UART_DEBUG_INSTANCE); 
    
    LL_USART_InitTypeDef USART1_InitStructure = {0};
    USART1_InitStructure.BaudRate = HW_UART_DEBUG_BAUDRATE; // 调试波特率
    USART1_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
    USART1_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART1_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
    USART1_InitStructure.Parity = LL_USART_PARITY_NONE;
    USART1_InitStructure.StopBits = LL_USART_STOPBITS_1;
    USART1_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;
    
    LL_USART_Init(HW_UART_DEBUG_INSTANCE , &USART1_InitStructure);
    LL_USART_ConfigAsyncMode(HW_UART_DEBUG_INSTANCE); 
    LL_USART_Enable(HW_UART_DEBUG_INSTANCE);


    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2) ; 

    LL_USART_Disable(HW_UART_ELRS_INSTANCE) ;

    LL_USART_InitTypeDef USART2_InitStructure = {0} ; 
    USART2_InitStructure.BaudRate = HW_UART2_BAUDRATE; // CRSF 波特率
    USART2_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
    USART2_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART2_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
    USART2_InitStructure.Parity = LL_USART_PARITY_NONE;
    USART2_InitStructure.StopBits = LL_USART_STOPBITS_1;
    USART2_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;

    LL_USART_Init(HW_UART_ELRS_INSTANCE , &USART2_InitStructure) ; 

    LL_USART_ConfigAsyncMode(HW_UART_ELRS_INSTANCE); 
    LL_USART_Enable(HW_UART_ELRS_INSTANCE);


    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3) ; 

    LL_USART_Disable(HW_UART_GPS_INSTANCE) ;

    LL_USART_InitTypeDef USART3_InitStructure = {0} ; 
    USART3_InitStructure.BaudRate = HW_UART_GPS_BAUDRATE; 
    USART3_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
    USART3_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART3_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
    USART3_InitStructure.Parity = LL_USART_PARITY_NONE;
    USART3_InitStructure.StopBits = LL_USART_STOPBITS_1;
    USART3_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;

    LL_USART_Init(HW_UART_GPS_INSTANCE , &USART3_InitStructure) ; 

    LL_USART_ConfigAsyncMode(HW_UART_GPS_INSTANCE); 
    LL_USART_Enable(HW_UART_GPS_INSTANCE);

}
