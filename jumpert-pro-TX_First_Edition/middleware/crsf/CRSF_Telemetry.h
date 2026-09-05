#ifndef __CRSF_TELEMETRY_H
#define __CRSF_TELEMETRY_H

#include "crsf/CRSF_Protocol.h"
#include "stdint.h"

typedef struct 
{
    uint16_t buffer[16] ; 
} crsf_raw_channels_t ;

void CRSF_Send_RC_Channel(const crsf_raw_channels_t* channels) ; 
void CRSF_Send_Frame(uint8_t sync_address, uint8_t type, uint8_t* payload, uint8_t payload_len) ; 

#endif
