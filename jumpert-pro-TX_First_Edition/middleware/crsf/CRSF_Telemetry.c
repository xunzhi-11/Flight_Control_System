#include "crsf/CRSF_Telemetry.h"
#include "crsf/CRSF_Protocol.h"
#include "utils/crc8.h"
#include "stdint.h"
#include "uart_ringbuf/uart1_ringbuf.h"
#include "tim/ll_tim3.h"
#include "stddef.h"

void CRSF_Send_Frame(uint8_t sync_address, uint8_t type, uint8_t* payload, uint8_t payload_len)
{
    uint8_t tx_frame[CRSF_FRAME_SIZE_MAX] ; 
    
    tx_frame[0] = sync_address ;            
    tx_frame[1] = payload_len + 2 ;         // 长度 = Type(1) + Payload + CRC(1)
    tx_frame[2] = type ;                   
    
    for(uint8_t i = 0 ; i < payload_len ; i++)
    {
        tx_frame[3 + i] = payload[i] ;
    }

    uint8_t crc = CRSF_Calc_CRC8(type, payload, payload_len) ;
    tx_frame[3 + payload_len] = crc ;
    
    UART1_Transmit_DMA_Async(tx_frame, payload_len + 4) ;
}

void CRSF_Send_RC_Channel(const crsf_raw_channels_t* channels)
{
    if(channels == NULL) return ; 

    uint8_t payload[CRSF_FRAME_PAYLOAD_LENGTH_RC_CHANNEL] ; 

    payload[0] = (uint8_t)(channels->buffer[0]) ; 
    payload[1] = (uint8_t)(channels->buffer[0] >> 8 | channels->buffer[1] << 3) ; 
    payload[2] = (uint8_t)(channels->buffer[1] >> 5 | channels->buffer[2] << 6) ; 
    payload[3] = (uint8_t)(channels->buffer[2] >> 2) ; 
    payload[4] = (uint8_t)(channels->buffer[2] >> 10 | channels->buffer[3] << 1) ; 
    payload[5] = (uint8_t)(channels->buffer[3] >> 7 | channels->buffer[4] << 4) ; 
    payload[6] = (uint8_t)(channels->buffer[4] >> 4 | channels->buffer[5] << 7) ; 
    payload[7] = (uint8_t)(channels->buffer[5] >> 1) ; 
    payload[8] = (uint8_t)(channels->buffer[5] >> 9 | channels->buffer[6] << 2) ; 
    payload[9] = (uint8_t)(channels->buffer[6] >> 6 | channels->buffer[7] << 5) ; 
    payload[10] = (uint8_t)(channels->buffer[7] >> 3) ; 
    payload[11] = (uint8_t)(channels->buffer[8]) ; 
    payload[12] = (uint8_t)(channels->buffer[8] >> 8 | channels->buffer[9] << 3) ; 
    payload[13] = (uint8_t)(channels->buffer[9] >> 5 | channels->buffer[10] << 6) ; 
    payload[14] = (uint8_t)(channels->buffer[10] >> 2) ; 
    payload[15] = (uint8_t)(channels->buffer[10] >> 10 | channels->buffer[11] << 1) ; 
    payload[16] = (uint8_t)(channels->buffer[11] >> 7 | channels->buffer[12] << 4) ; 
    payload[17] = (uint8_t)(channels->buffer[12] >> 4 | channels->buffer[13] << 7) ; 
    payload[18] = (uint8_t)(channels->buffer[13] >> 1) ; 
    payload[19] = (uint8_t)(channels->buffer[13] >> 9 | channels->buffer[14] << 2) ; 
    payload[20] = (uint8_t)(channels->buffer[14] >> 6 | channels->buffer[15] << 5) ; 
    payload[21] = (uint8_t)(channels->buffer[15] >> 3) ; 

    CRSF_Send_Frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 
                    CRSF_FRAME_TYPE_RC_CHANNELS, payload, 
                    CRSF_FRAME_PAYLOAD_LENGTH_RC_CHANNEL) ;

}



