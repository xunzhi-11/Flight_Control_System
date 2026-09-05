#include "dshot_protocol.h"
#include "stdint.h"
#include "stdbool.h"

uint16_t DShot_Encode_Packet(float throttle_percent, bool request_telemetry)
{
    uint16_t throttle_int = 0;

    //  钳位
    if (throttle_percent < 0.0f) throttle_percent = 0.0f;
    if (throttle_percent > 1.0f) throttle_percent = 1.0f;

    // 映射到有效 DShot 区间 
    if (throttle_percent == 0.0f) 
    {
        throttle_int = 0; // 发送 0 让电调停止并进入待命状态
    } 
    else 
    {
        // DShot 实际油门范围: 48 (0%) 到 2047 (100%)
        // 2047 - 48 = 1999
        throttle_int = 48 + (uint16_t)(throttle_percent * 1999.0f + 0.5f);
    }

    // 组装数据与 CRC 
    uint16_t packet = (throttle_int << 1) | (request_telemetry ? 1 : 0);
    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    
    return (packet << 4) | csum;
}