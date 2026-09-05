#include "crsf/CRSF_Telemetry.h"
#include "crsf/CRSF_Protocol.h"
#include "utils/crc8/crc8.h"
#include "stdint.h"
#include "uart/ll_uart.h"
#include "data_structure_types.h"



static void CRSF_Send_Telemetry_Frame(uint8_t type, uint8_t* payload, uint8_t payload_len)
{
    uint8_t tx_frame[CRSF_FRAME_SIZE_MAX]; 
    
    tx_frame[0] = CRSF_FRAME_HEADER;       
    //  Type(1) + Payload + CRC(1)
    tx_frame[1] = payload_len + 2;         
    tx_frame[2] = type;                   
    
    for(uint8_t i = 0; i < payload_len; i++)
    {
        tx_frame[3 + i] = payload[i];
    }
    uint8_t crc = CRSF_Calc_CRC8(type, payload, payload_len);
    tx_frame[3 + payload_len] = crc;
    
    //  (头帧)1 + (长度)1 + (Type)1 + payload_len + (CRC)1
    UART2_Transmit_DMA_Async(tx_frame, payload_len + 4);
}

void CRSF_Send_Battery(const crsf_telemetry_battery_t* batt_data)
{
    if (batt_data == NULL) return; 

    uint8_t payload[CRSF_FRAME_PAYLOAD_LENGTH_BATTERY]; 
    
    // 电压 
    payload[0] = (batt_data->voltage_01v >> 8) & 0xFF; 
    payload[1] = (batt_data->voltage_01v >> 0) & 0xFF; 
    
    // 电流 
    payload[2] = (batt_data->current_01a >> 8) & 0xFF; 
    payload[3] = (batt_data->current_01a >> 0) & 0xFF; 
    
    // 容量 
    payload[4] = (batt_data->capacity_mah >> 16) & 0xFF; 
    payload[5] = (batt_data->capacity_mah >> 8)  & 0xFF; 
    payload[6] = (batt_data->capacity_mah >> 0)  & 0xFF; 
    
    // 剩余电量 
    payload[7] = batt_data->remaining_pct; 

    CRSF_Send_Telemetry_Frame(CRSF_FRAME_TYPE_BATTERY_SENSOR, payload, CRSF_FRAME_PAYLOAD_LENGTH_BATTERY);
}

void CRSF_Send_GPS(const crsf_telemetry_gps_t* gps_data)
{
   if (gps_data == NULL) return;
    
    uint8_t payload[CRSF_FRAME_PAYLOAD_LENGTH_GPS]; 
    
    // 纬度 Latitude 
    payload[0] = (gps_data->latitude >> 24) & 0xFF;
    payload[1] = (gps_data->latitude >> 16) & 0xFF;
    payload[2] = (gps_data->latitude >> 8)  & 0xFF;
    payload[3] = (gps_data->latitude >> 0)  & 0xFF;
    
    // 经度 Longitude 
    payload[4] = (gps_data->longitude >> 24) & 0xFF;
    payload[5] = (gps_data->longitude >> 16) & 0xFF;
    payload[6] = (gps_data->longitude >> 8)  & 0xFF;
    payload[7] = (gps_data->longitude >> 0)  & 0xFF;
    
    // 地速 Ground Speed 
    payload[8] = (gps_data->ground_speed >> 8) & 0xFF;
    payload[9] = (gps_data->ground_speed >> 0) & 0xFF;
    
    // 航向 Heading 
    payload[10] = (gps_data->heading >> 8) & 0xFF;
    payload[11] = (gps_data->heading >> 0) & 0xFF;
    
    // 高度 Altitude
    uint16_t payload_alt = gps_data->altitude_m + 1000;
    payload[12] = (payload_alt >> 8) & 0xFF;
    payload[13] = (payload_alt >> 0) & 0xFF;
    
    // 卫星数量 Satellites 
    payload[14] = gps_data->satellites;
   
    CRSF_Send_Telemetry_Frame(CRSF_FRAME_TYPE_GPS, payload, CRSF_FRAME_PAYLOAD_LENGTH_GPS);
}

void CRSF_Send_Attitude(const crsf_telemetry_attitude_t* att_data)
{
    if(att_data == NULL) return ; 

    uint8_t payload[CRSF_FRAME_PAYLOAD_LENGTH_ATTITUDE] ; 

    payload[0] = (att_data->pitch_rad_10k >> 8) & 0xFF ; 
    payload[1] = (att_data->pitch_rad_10k >> 0) & 0xFF ; 
    payload[2] = (att_data->roll_rad_10k >> 8) & 0xFF ; 
    payload[3] = (att_data->roll_rad_10k >> 0) & 0xFF ; 
    payload[4] = (att_data->yaw_rad_10k >> 8) & 0xFF ; 
    payload[5] = (att_data->yaw_rad_10k >> 0) & 0xFF ; 
    
    CRSF_Send_Telemetry_Frame(CRSF_FRAME_TYPE_ATTITUDE , payload , CRSF_FRAME_PAYLOAD_LENGTH_ATTITUDE) ; 
}




