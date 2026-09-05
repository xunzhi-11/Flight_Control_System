#include "crsf/CRSF_Parser.h"
#include "crsf/CRSF_Protocol.h"
#include "utils/crc8.h"
#include "stdio.h"
#include "string.h"

typedef enum {
    CRSF_STATE_SYNC = 0,
    CRSF_STATE_LENGTH = 1 ,
    CRSF_STATE_TYPE = 2 ,
    CRSF_STATE_PAYLOAD = 3 ,
    CRSF_STATE_CRC = 4
} CRSF_State_t ;


typedef struct {
    CRSF_State_t state ;
    uint8_t expected_length ;
    uint8_t frame_type ;
    uint8_t payload_buffer[CRSF_FRAME_SIZE_MAX] ; 
    uint8_t payload_idx ;
    uint8_t device_addr ;   
    uint8_t last_received_param_id ;
    uint8_t last_param_payload[CRSF_FRAME_SIZE_MAX] ; 
    uint8_t last_param_payload_len ;
    crsf_device_info_t device_info ;  
    crsf_link_statistics_t link_stats ;
    crsf_telemetry_battery_t batt ; 
    crsf_telemetry_gps_t gps ; 
    crsf_telemetry_attitude_t atti ; 
} CRSF_Parser_Context_t ;

CRSF_Parser_Context_t crsf_ctx = {0} ;

static uint32_t s_link_stats_seq = 0 ;
static uint32_t s_battery_seq = 0 ;
static uint32_t s_gps_seq = 0 ;
static uint32_t s_module_rx_seq = 0 ;



static void CRSF_Decode_LINK_STATISTICS(uint8_t* crsf_frame_buffer , crsf_link_statistics_t* crsf_link_statistics) 
{
    crsf_link_statistics->uplink_RSSI_1 = crsf_frame_buffer[0] ;
    crsf_link_statistics->uplink_RSSI_2 = crsf_frame_buffer[1] ;
    crsf_link_statistics->uplink_LQ = crsf_frame_buffer[2] ;
    crsf_link_statistics->uplink_SNR = (int8_t)crsf_frame_buffer[3] ;
    crsf_link_statistics->active_antenna = crsf_frame_buffer[4] ;
    crsf_link_statistics->rf_Mode = crsf_frame_buffer[5] ;
    crsf_link_statistics->uplink_TX_Power = crsf_frame_buffer[6] ;
    crsf_link_statistics->downlink_RSSI = crsf_frame_buffer[7] ;
    crsf_link_statistics->downlink_LQ = crsf_frame_buffer[8] ;
    crsf_link_statistics->downlink_SNR = (int8_t)crsf_frame_buffer[9] ;
}

static void CRSF_Decode_Batt(uint8_t* csrf_frame_buffer , crsf_telemetry_battery_t* crsf_batt_data)
{
    crsf_batt_data->voltage_01v = (uint16_t)(csrf_frame_buffer[0] << 8 | csrf_frame_buffer[1]) ;
    crsf_batt_data->current_01a = (uint16_t)(csrf_frame_buffer[2] << 8 | csrf_frame_buffer[3]) ;
    crsf_batt_data->capacity_mah = (uint32_t)(csrf_frame_buffer[4] << 16 | csrf_frame_buffer[5] << 8 | csrf_frame_buffer[6]) ;
    crsf_batt_data->remaining_pct = (uint8_t)(csrf_frame_buffer[7]) ; 
}

static void CRSF_Decode_GPS(uint8_t* csrf_frame_buffer , crsf_telemetry_gps_t* crsf_gps_data)
{
    crsf_gps_data->latitude = (int32_t)( csrf_frame_buffer[0] << 24 | csrf_frame_buffer[1] << 16 | csrf_frame_buffer[2] << 8 | csrf_frame_buffer[3]) ;
    crsf_gps_data->longitude = (int32_t)(csrf_frame_buffer[4] << 24 | csrf_frame_buffer[5] << 16 | csrf_frame_buffer[6] << 8 | csrf_frame_buffer[7]) ;
    crsf_gps_data->ground_speed = (uint16_t)(csrf_frame_buffer[8] << 8 | csrf_frame_buffer[9]) ;
    crsf_gps_data->heading = (uint16_t)(csrf_frame_buffer[10] << 8 | csrf_frame_buffer[11]) ;
    crsf_gps_data->alttitude_m = (uint16_t)(csrf_frame_buffer[12] << 8 | csrf_frame_buffer[13]) ;
    crsf_gps_data->satellites = (uint8_t)(csrf_frame_buffer[14]) ; 
}

static void CRSF_Decode_Atti(uint8_t* crsf_frame_buffer ,crsf_telemetry_attitude_t* crsf_atti_data)
{
    crsf_atti_data->pitch_rad_10k = (uint16_t)(crsf_frame_buffer[0] << 8 | crsf_frame_buffer[1]) ;
    crsf_atti_data->roll_rad_10k = (uint16_t)(crsf_frame_buffer[2] << 8 | crsf_frame_buffer[3]) ; 
    crsf_atti_data->yaw_rad_10k = (uint16_t)(crsf_frame_buffer[4] << 8 |crsf_frame_buffer[5]) ; 
}
static void CRSF_Decode_DeviceInfo(uint8_t* payload, uint8_t length, crsf_device_info_t* info)
{
    uint8_t idx = 2 ;
    uint8_t str_idx = 0 ;

    // 提取设备名称 
    while(idx < length && payload[idx] != '\0' && str_idx < 31) {
        info->device_name[str_idx++] = payload[idx++] ;
    }
    info->device_name[str_idx] = '\0' ;
    idx++ ; 

    // 提取序列号 (固定 4 字节)
    if (idx + 4 <= length) {
        info->serial_number = (uint32_t)(payload[idx] << 24 | payload[idx+1] << 16 | payload[idx+2] << 8 | payload[idx+3]) ;
        idx += 4 ;
    }

    // 提取硬件版本 (固定 4 字节)
    if (idx + 4 <= length) {
        info->hardware_version = (uint32_t)(payload[idx] << 24 | payload[idx+1] << 16 | payload[idx+2] << 8 | payload[idx+3]) ;
        idx += 4 ;
    }

    // 提取软件版本 (固定 4 字节，存入数组)
    if (idx + 4 <= length) {
        info->software_version[0] = payload[idx++] ;
        info->software_version[1] = payload[idx++] ;
        info->software_version[2] = payload[idx++] ;
        info->software_version[3] = payload[idx++] ;
    }

    // 提取参数总数和版本 (各 1 字节)
    if (idx < length) info->parameter_count = payload[idx++] ;
    if (idx < length) info->parameter_version = payload[idx++] ;
}

void CRSF_Parser_Byte(uint8_t byte)
{
    switch (crsf_ctx.state)
    {
        case CRSF_STATE_SYNC:
            // 允许解析发给 遥控器(0xEA)、飞控(0xC8) 或 广播(0x00) 的包
            if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || 
                byte == CRSF_ADDRESS_RADIO_TRANSMITTER || 
                byte == CRSF_ADDRESS_BROADCAST)
            {
                crsf_ctx.device_addr = byte ; 
                crsf_ctx.state = CRSF_STATE_LENGTH ;
            }
            break ;

        case CRSF_STATE_LENGTH:
            if (byte >= 2 && byte <= CRSF_FRAME_SIZE_MAX) 
            {
                crsf_ctx.expected_length = byte ;
                crsf_ctx.state = CRSF_STATE_TYPE ;
            }
            else
            {
                crsf_ctx.state = CRSF_STATE_SYNC ;
            }
            break ;

        case CRSF_STATE_TYPE :
            crsf_ctx.frame_type = byte ; 
            crsf_ctx.expected_length -- ; 
            crsf_ctx.payload_idx = 0 ;
            crsf_ctx.state = CRSF_STATE_PAYLOAD ; 
        break ;

        case CRSF_STATE_PAYLOAD :
            crsf_ctx.payload_buffer[crsf_ctx.payload_idx ++] = byte ; 
            crsf_ctx.expected_length -- ; 

            if(crsf_ctx.expected_length == 1)
            {
                crsf_ctx.state = CRSF_STATE_CRC ; 
            }
        break ;
        
      case CRSF_STATE_CRC :
            // 计算出的 CRC 与 接收到的 CRC 一致
            if (CRSF_Calc_CRC8(crsf_ctx.frame_type, crsf_ctx.payload_buffer, crsf_ctx.payload_idx) == byte) 
            {
                s_module_rx_seq++ ;

                if (crsf_ctx.device_addr == CRSF_ADDRESS_RADIO_TRANSMITTER || 
                    crsf_ctx.device_addr == CRSF_ADDRESS_BROADCAST)
                {
                    switch(crsf_ctx.frame_type)
                    {
                        case CRSF_FRAME_TYPE_DEVICE_INFO:
                            CRSF_Decode_DeviceInfo(crsf_ctx.payload_buffer, crsf_ctx.payload_idx, &crsf_ctx.device_info) ;
                            break ;
                            
                        case CRSF_FRAME_TYPE_PARAMETER_SETTINGS_ENTRY:
                            // 记录 ID
                            crsf_ctx.last_received_param_id = crsf_ctx.payload_buffer[2] ;
                            // 将整段 Payload 拷贝到缓存，供 App 层提取
                            memcpy(crsf_ctx.last_param_payload, crsf_ctx.payload_buffer, crsf_ctx.payload_idx) ;
                            crsf_ctx.last_param_payload_len = crsf_ctx.payload_idx ;
                            break ;
                    }
                }
                switch(crsf_ctx.frame_type)
                {
                    case CRSF_FRAME_TYPE_BATTERY_SENSOR:
                        CRSF_Decode_Batt(crsf_ctx.payload_buffer, &crsf_ctx.batt) ;
                        s_battery_seq++ ;
                        break ;
                    case CRSF_FRAME_TYPE_LINK_STATISTICS:
                        CRSF_Decode_LINK_STATISTICS(crsf_ctx.payload_buffer, &crsf_ctx.link_stats) ;
                        s_link_stats_seq++ ;
                        break ;
                    case CRSF_FRAME_TYPE_GPS:
                        CRSF_Decode_GPS(crsf_ctx.payload_buffer, &crsf_ctx.gps) ;
                        s_gps_seq++ ;
                        break ;
                    case CRSF_FRAME_TYPE_ATTITUDE:
                        CRSF_Decode_Atti(crsf_ctx.payload_buffer, &crsf_ctx.atti) ;
                        break ;
                }
            }

            crsf_ctx.state = CRSF_STATE_SYNC ;
        break ;

        default:
        break ;
    }
}


uint8_t CRSF_Get_Parameter_Count(void)
{
    return crsf_ctx.device_info.parameter_count ;
}

uint8_t CRSF_Get_Last_Received_Param_ID(void)
{
    return crsf_ctx.last_received_param_id ;
}

crsf_device_info_t* CRSF_Get_Device_Info_Ptr(void)
{
    return &crsf_ctx.device_info ;
}

uint8_t CRSF_Get_Last_Param_Data(uint8_t* buffer_out)
{
    if (buffer_out != NULL && crsf_ctx.last_param_payload_len > 0)
    {
        memcpy(buffer_out, crsf_ctx.last_param_payload, crsf_ctx.last_param_payload_len) ;
        return crsf_ctx.last_param_payload_len ;
    }
    return 0 ;
}

crsf_link_statistics_t* CRSF_Get_Link_Stats_Ptr(void)
{
    return &crsf_ctx.link_stats ;
}

crsf_telemetry_battery_t* CRSF_Get_Battery_Ptr(void)
{
    return &crsf_ctx.batt ;
}

crsf_telemetry_gps_t* CRSF_Get_GPS_Ptr(void)
{
    return &crsf_ctx.gps ;
}

uint32_t CRSF_Get_Link_Stats_Seq(void)
{
    return s_link_stats_seq ;
}

uint32_t CRSF_Get_Battery_Seq(void)
{
    return s_battery_seq ;
}

uint32_t CRSF_Get_GPS_Seq(void)
{
    return s_gps_seq ;
}

uint32_t CRSF_Get_Module_Rx_Seq(void)
{
    return s_module_rx_seq ;
}