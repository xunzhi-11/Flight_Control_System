#include "utils/crc8/crc8.h"
#include "stdint.h"

#define CRSF_FRANE_CRC8_CALI_POLY 0xD5

uint8_t CRSF_Calc_CRC8(uint8_t type , uint8_t* payload , uint8_t payload_len )
{
    uint8_t crc = 0 ; 

    crc ^= type ;
    for(uint8_t j = 0; j < 8; j++) 
    {
        if(crc & 0x80) 
        {
            crc = (crc << 1) ^ CRSF_FRANE_CRC8_CALI_POLY;
        }
        else 
        {
            crc <<= 1;
        }
    }

    for(uint8_t i = 0 ; i < payload_len ; i++)
    {
        crc ^= payload[i] ; 
        
        for(uint8_t j = 0 ; j < 8 ; j++)
        {
            if(crc & 0X80)
            {
                crc = (crc << 1) ^ CRSF_FRANE_CRC8_CALI_POLY ;  
            }
            else
            {
                crc <<= 1 ;
            }
        }
    }

    return crc ; 
}