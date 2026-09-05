#include "crsf/CRSF_Parser.h"
#include "crsf/CRSF_Protocol.h"
#include "utils/crc8/crc8.h"
#include "sys_software_config.h"
#include "app_core/app_motor_unlocker.h"

typedef enum 
{
    CRSF_STATE_SYNC = 0,
    CRSF_STATE_LENGTH = 1 ,
    CRSF_STATE_TYPE = 2 ,
    CRSF_STATE_PAYLOAD = 3 ,
    CRSF_STATE_CRC = 4
} CRSF_State_t;

typedef struct 
{
    CRSF_State_t state;
    uint8_t expected_length;
    uint8_t frame_type;
    uint8_t payload_buffer[CRSF_FRAME_SIZE_MAX]; 
    uint8_t payload_idx;
    // 解析后的有效数据存放在这里，供主循环读取
    crsf_channels_t channels;
    crsf_link_statistics_t link_stats;
} CRSF_Parser_Context_t;

CRSF_Parser_Context_t crsf_ctx = {0} ;

/* 供 4kHz 控制环读取：parser 写非激活缓冲后翻转下标 */
static crsf_channels_t s_rc_control_buf[2];
static volatile uint8_t s_rc_control_active = 0;

/* ch0-ch3 摇杆一阶低通：y += alpha * (x - y)，在 CRSF 帧率下更新 */
#define RC_STICK_FILTER_CH_COUNT        4U
#define RC_STICK_LPF_ALPHA              0.4f
#define CRSF_CHANNEL_RAW_MIN            172U
#define CRSF_CHANNEL_RAW_MAX            1811U

static float s_rc_stick_filtered[RC_STICK_FILTER_CH_COUNT];
static uint8_t s_rc_stick_filter_init = 0U;

static uint16_t CRSF_ClampChannelRaw(float value)
{
    if (value < (float)CRSF_CHANNEL_RAW_MIN) {
        return CRSF_CHANNEL_RAW_MIN;
    }
    if (value > (float)CRSF_CHANNEL_RAW_MAX) {
        return CRSF_CHANNEL_RAW_MAX;
    }
    return (uint16_t)(value + 0.5f);
}

static void CRSF_ApplyStickChannelFilter(crsf_channels_t *channels)
{
    uint16_t *stick_ch[RC_STICK_FILTER_CH_COUNT] = {
        &channels->ch0,
        &channels->ch1,
        &channels->ch2,
        &channels->ch3,
    };

    if (channels == NULL) {
        return;
    }

    if (!s_rc_stick_filter_init) {
        for (uint8_t i = 0U; i < RC_STICK_FILTER_CH_COUNT; i++) {
            s_rc_stick_filtered[i] = (float)*stick_ch[i];
            *stick_ch[i] = CRSF_ClampChannelRaw(s_rc_stick_filtered[i]);
        }
        s_rc_stick_filter_init = 1U;
        return;
    }

    for (uint8_t i = 0U; i < RC_STICK_FILTER_CH_COUNT; i++) {
        const float input = (float)*stick_ch[i];
        s_rc_stick_filtered[i] += RC_STICK_LPF_ALPHA * (input - s_rc_stick_filtered[i]);
        *stick_ch[i] = CRSF_ClampChannelRaw(s_rc_stick_filtered[i]);
    }
}

static void CRSF_PublishControlSnapshot(const crsf_channels_t *src)
{
    uint8_t inactive = s_rc_control_active ^ 1u;
    s_rc_control_buf[inactive] = *src;
    s_rc_control_active = inactive;
}

static void CRSF_Decode_RC_CHANNELS(uint8_t* crsf_frame_buffer , crsf_channels_t* crsf_channels)
{
    crsf_channels->ch0  = ((crsf_frame_buffer[0]  >> 0 | crsf_frame_buffer[1]  << 8) ) & 0x07FF;
    crsf_channels->ch1  = ((crsf_frame_buffer[1]  >> 3 | crsf_frame_buffer[2]  << 5) ) & 0x07FF;
    crsf_channels->ch2  = ((crsf_frame_buffer[2]  >> 6 | crsf_frame_buffer[3]  << 2 | crsf_frame_buffer[4] << 10) ) & 0x07FF;
    crsf_channels->ch3  = ((crsf_frame_buffer[4]  >> 1 | crsf_frame_buffer[5]  << 7) ) & 0x07FF;
    crsf_channels->ch4  = ((crsf_frame_buffer[5]  >> 4 | crsf_frame_buffer[6]  << 4) ) & 0x07FF;
    crsf_channels->ch5  = ((crsf_frame_buffer[6]  >> 7 | crsf_frame_buffer[7]  << 1 | crsf_frame_buffer[8] <<  9) ) & 0x07FF;
    crsf_channels->ch6  = ((crsf_frame_buffer[8]  >> 2 | crsf_frame_buffer[9]  << 6) ) & 0x07FF;
    crsf_channels->ch7  = ((crsf_frame_buffer[9]  >> 5 | crsf_frame_buffer[10] << 3) ) & 0x07FF;
    crsf_channels->ch8  = ((crsf_frame_buffer[11] >> 0 | crsf_frame_buffer[12] << 8) ) & 0x07FF;
    crsf_channels->ch9  = ((crsf_frame_buffer[12] >> 3 | crsf_frame_buffer[13] << 5) ) & 0x07FF;
    crsf_channels->ch10 = ((crsf_frame_buffer[13] >> 6 | crsf_frame_buffer[14] << 2 | crsf_frame_buffer[15] << 10) ) & 0x07FF;
    crsf_channels->ch11 = ((crsf_frame_buffer[15] >> 1 | crsf_frame_buffer[16] << 7) ) & 0x07FF;
    crsf_channels->ch12 = ((crsf_frame_buffer[16] >> 4 | crsf_frame_buffer[17] << 4) ) & 0x07FF;
    crsf_channels->ch13 = ((crsf_frame_buffer[17] >> 7 | crsf_frame_buffer[18] << 1 | crsf_frame_buffer[19] <<  9) ) & 0x07FF;
    crsf_channels->ch14 = ((crsf_frame_buffer[19] >> 2 | crsf_frame_buffer[20] << 6) ) & 0x07FF;
    crsf_channels->ch15 = ((crsf_frame_buffer[20] >> 5 | crsf_frame_buffer[21] << 3) ) & 0x07FF;
}

uint16_t CRSF_Channel_To_PWM(uint16_t raw_value) 
{
    if(raw_value < 172) raw_value = 172;
    if(raw_value > 1811) raw_value = 1811;
    
    // 乘 1000 除以 1639，用 32 位防溢出
    return 1000 + ((uint32_t)(raw_value - 172) * 1000) / 1639;
}

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

void CRSF_Parser_Byte(uint8_t byte)
{
    switch (crsf_ctx.state)
    {
        case CRSF_STATE_SYNC :
            if(byte == CRSF_FRAME_HEADER)
            {
                crsf_ctx.state = CRSF_STATE_LENGTH ; 
            }
        break;

        case CRSF_STATE_LENGTH :
            if(byte < CRSF_FRAME_SIZE_MINIMAL || byte > CRSF_FRAME_SIZE_MAX)
            {
                crsf_ctx.state = CRSF_STATE_SYNC ; 
            }
            else
            {
                crsf_ctx.expected_length = byte ; 
                crsf_ctx.state = CRSF_STATE_TYPE ; 
            }
        break;

        case CRSF_STATE_TYPE :
            crsf_ctx.frame_type = byte ; 
            crsf_ctx.expected_length -- ; 
            crsf_ctx.payload_idx = 0 ;
            crsf_ctx.state = CRSF_STATE_PAYLOAD ; 
        break;

        case CRSF_STATE_PAYLOAD :
            crsf_ctx.payload_buffer[crsf_ctx.payload_idx ++] = byte ; 
            crsf_ctx.expected_length -- ; 

            if(crsf_ctx.expected_length == 1)
            {
                crsf_ctx.state = CRSF_STATE_CRC ; 
            }
        break;
        
        case CRSF_STATE_CRC :
            if (CRSF_Calc_CRC8(crsf_ctx.frame_type, crsf_ctx.payload_buffer, crsf_ctx.payload_idx) == byte) 
            {
                if (crsf_ctx.frame_type == CRSF_FRAME_TYPE_RC_CHANNELS) 
                {
                    crsf_channels_t decoded;

                    CRSF_Decode_RC_CHANNELS(crsf_ctx.payload_buffer, &decoded);
                    CRSF_ApplyStickChannelFilter(&decoded);

                    crsf_ctx.channels = decoded;
                    CRSF_PublishControlSnapshot(&decoded);
                    Safety_RC_FeedDog() ; 
                } 
                else if (crsf_ctx.frame_type == CRSF_FRAME_TYPE_LINK_STATISTICS) 
                {
                    CRSF_Decode_LINK_STATISTICS(crsf_ctx.payload_buffer, &crsf_ctx.link_stats);
                }
            }

            crsf_ctx.state = CRSF_STATE_SYNC;
        break;

        default:
        break;
    }
}

void CRSF_Get_Channels(crsf_channels_t* out_data)
{
    if (out_data == NULL) {
        return;
    }

    *out_data = crsf_ctx.channels;
}

void CRSF_ReadChannels_Control(crsf_channels_t* out_data)
{
    if (out_data == NULL) 
    {
        return;
    }

    uint8_t idx = s_rc_control_active;
    *out_data = s_rc_control_buf[idx];

    /* 拷贝期间若发生翻转，再读一次最新缓冲 */
    if (idx != s_rc_control_active) {
        *out_data = s_rc_control_buf[s_rc_control_active];
    }
}