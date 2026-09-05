#include "elrs/elrs_manager.h"
#include "crsf/CRSF_Telemetry.h"



void CRSF_Send_Ping(void)
{
    uint8_t payload[2] ;
    
    // 逻辑寻址
    payload[0] = CRSF_ADDRESS_CRSF_TRANSMITTER ;  // 目标：高频头 (0xEE)
    payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER ; // 来源：主板 (0xEA)
    
    // 物理地址发给高频头 (0xEE)，类型为 Ping (0x28)
    CRSF_Send_Frame(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAME_TYPE_DEVICE_PING, payload, 2) ;
}

// 发送参数读取请求 (0x2B)
void CRSF_Request_Parameter(uint8_t param_id)
{
    uint8_t payload[4] ;
    
    // 逻辑寻址
    payload[0] = CRSF_ADDRESS_CRSF_TRANSMITTER ;  // 目标：高频头 (0xEE)
    payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER ; // 来源：主板 (0xEA)
    
    payload[2] = param_id ;  // 请求的参数 ID (1 到 20)
    payload[3] = 0 ;         // Chunk ID 固定为 0 (从头开始读这个参数)
    
    // 物理地址发给高频头 (0xEE)，类型为 Parameter Read (0x2B)
    CRSF_Send_Frame(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAME_TYPE_PARAMETER_READ, payload, 4) ;
}
// 发送参数写入请求 (0x2D)
static void CRSF_Write_Parameter(uint8_t param_id, uint8_t value)
{
    uint8_t payload[4] ; 
    
    // 逻辑寻址
    payload[0] = CRSF_ADDRESS_CRSF_TRANSMITTER ;  // 目标：高频头 (0xEE)
    payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER ; // 来源：主板 (0xEA)
    
    // 写入内容
    payload[2] = param_id ;  // 要修改的参数 ID
    payload[3] = value ;     
    
    // 发送 0x2D (PARAMETER_WRITE)
    CRSF_Send_Frame(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAME_TYPE_PARAMETER_WRITE, payload, 4) ;
}

void ELRS_Set_Param(ELRS_Param_ID_e param_id, uint8_t value_index)
{
    CRSF_Write_Parameter((uint8_t)param_id, value_index) ;
}

void ELRS_Trigger_Command(ELRS_Param_ID_e cmd_id)
{
    CRSF_Write_Parameter((uint8_t)cmd_id, 1) ;
}