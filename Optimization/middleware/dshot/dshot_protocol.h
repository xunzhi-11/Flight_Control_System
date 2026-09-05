#ifndef __DSHOT_PROTOCOL_H
#define __DSHOT_PROTOCOL_H

#include "stdint.h"
#include "stdbool.h"

uint16_t DShot_Encode_Packet(float throttle_percent, bool request_telemetry) ; 

#endif
