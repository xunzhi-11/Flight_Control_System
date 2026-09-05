#include "utils/joycal.h"
#include "flash/ll_flash.h"
#include "system_hardware_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stddef.h"
#include "string.h"

#define RC_CHANNEL_VALUE_MAX 1811
#define RC_CHANNEL_VALUE_MIN 172

#define JOY_CALIB_MAGIC         0x4A4F5943UL
#define JOY_CALIB_VERSION       1U
#define JOY_CALIB_MIN_SPAN      100U
#define JOY_CALIB_ADC_MAX       4095U

typedef struct {
    uint32_t magic ;
    uint16_t version ;
    uint16_t channel_count ;
    uint16_t min[4] ;
    uint16_t center[4] ;
    uint16_t max[4] ;
    uint16_t deadband[4] ;
    uint32_t crc32 ;
} joy_calib_flash_record_t ;

static JoyCalib_Channel_t calib_data[4] = {
    {1000, 2048, 3000, 15}, /* Roll */
    {1050, 2010, 3100, 15}, /* Pitch */
    {1100, 2050, 2900, 0},  /* Throttle */
    {1020, 2040, 3050, 15}  /* Yaw */
} ;

static const JoyCalib_Channel_t s_default_calib[4] = {
    {1000, 2048, 3000, 15}, /* Roll */
    {1050, 2010, 3100, 15}, /* Pitch */
    {1100, 2050, 2900, 0},  /* Throttle */
    {1020, 2040, 3050, 15}  /* Yaw */
} ;

static uint32_t joy_calib_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL ;

    for (uint32_t i = 0 ; i < len ; i++)
    {
        crc ^= data[i] ;
        for (uint8_t bit = 0 ; bit < 8U ; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL ;
            }
            else
            {
                crc >>= 1 ;
            }
        }
    }

    return ~crc ;
}

static void calib_to_record(const JoyCalib_Channel_t data[4], joy_calib_flash_record_t *record)
{
    memset(record, 0, sizeof(*record)) ;
    record->magic = JOY_CALIB_MAGIC ;
    record->version = JOY_CALIB_VERSION ;
    record->channel_count = JOY_CALIB_CHANNEL_COUNT ;

    for (uint8_t i = 0 ; i < JOY_CALIB_CHANNEL_COUNT ; i++)
    {
        record->min[i] = data[i].min ;
        record->center[i] = data[i].center ;
        record->max[i] = data[i].max ;
        record->deadband[i] = data[i].deadband ;
    }

    record->crc32 = joy_calib_crc32((const uint8_t *)record,
                                    (uint32_t)offsetof(joy_calib_flash_record_t, crc32)) ;
}

static bool record_to_calib(const joy_calib_flash_record_t *record, JoyCalib_Channel_t out[4])
{
    if (record->magic != JOY_CALIB_MAGIC)
    {
        return false ;
    }

    if (record->version != JOY_CALIB_VERSION)
    {
        return false ;
    }

    if (record->channel_count != JOY_CALIB_CHANNEL_COUNT)
    {
        return false ;
    }

    if (record->crc32 != joy_calib_crc32((const uint8_t *)record,
                                         (uint32_t)offsetof(joy_calib_flash_record_t, crc32)))
    {
        return false ;
    }

    for (uint8_t i = 0 ; i < JOY_CALIB_CHANNEL_COUNT ; i++)
    {
        out[i].min = record->min[i] ;
        out[i].center = record->center[i] ;
        out[i].max = record->max[i] ;
        out[i].deadband = record->deadband[i] ;
    }

    return JoyCalib_Is_Valid(out) ;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
}

static uint16_t clamp_channels(long val)
{
    if (val < RC_CHANNEL_VALUE_MIN) return RC_CHANNEL_VALUE_MIN ;
    if (val > RC_CHANNEL_VALUE_MAX) return RC_CHANNEL_VALUE_MAX ;
    return (uint16_t)val ;
}

void JoyCalib_Get_Default(JoyCalib_Channel_t out[JOY_CALIB_CHANNEL_COUNT])
{
    memcpy(out, s_default_calib, sizeof(s_default_calib)) ;
}

void JoyCalib_Get_Current(JoyCalib_Channel_t out[JOY_CALIB_CHANNEL_COUNT])
{
    memcpy(out, calib_data, sizeof(calib_data)) ;
}

void JoyCalib_Apply(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT])
{
    memcpy(calib_data, data, sizeof(calib_data)) ;
}

bool JoyCalib_Is_Valid(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT])
{
    for (uint8_t i = 0 ; i < JOY_CALIB_CHANNEL_COUNT ; i++)
    {
        const JoyCalib_Channel_t *ch = &data[i] ;

        if (ch->min >= ch->center || ch->center >= ch->max)
        {
            return false ;
        }

        if ((ch->max - ch->min) < JOY_CALIB_MIN_SPAN)
        {
            return false ;
        }

        if (ch->max > JOY_CALIB_ADC_MAX)
        {
            return false ;
        }
    }

    return true ;
}

bool JoyCalib_Load(void)
{
    const joy_calib_flash_record_t *record =
        (const joy_calib_flash_record_t *)HW_FLASH_JOY_CALIB_ADDR ;

    JoyCalib_Channel_t loaded[4] ;

    if (!record_to_calib(record, loaded))
    {
        JoyCalib_Get_Default(loaded) ;
        JoyCalib_Apply(loaded) ;
        return false ;
    }

    JoyCalib_Apply(loaded) ;
    return true ;
}

bool JoyCalib_Save(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT])
{
    joy_calib_flash_record_t record ;
    uint32_t words[(sizeof(joy_calib_flash_record_t) + 3U) / 4U] ;
    bool ok = false ;

    if (!JoyCalib_Is_Valid(data))
    {
        return false ;
    }

    calib_to_record(data, &record) ;
    memcpy(words, &record, sizeof(record)) ;

    vTaskSuspendAll() ;

    if (LL_Flash_Unlock())
    {
        if (LL_Flash_EraseSector(HW_FLASH_JOY_CALIB_SECTOR))
        {
            ok = LL_Flash_ProgramWords(HW_FLASH_JOY_CALIB_ADDR,
                                       words,
                                       (uint32_t)(sizeof(record) / 4U)) ;
        }

        LL_Flash_Lock() ;
    }

    xTaskResumeAll() ;

    if (ok)
    {
        JoyCalib_Apply(data) ;
    }

    return ok ;
}

uint16_t Process_Joystick_Channel(uint8_t ch_index, uint16_t raw_adc)
{
    if (ch_index >= JOY_CALIB_CHANNEL_COUNT)
    {
        return 992U ;
    }

    JoyCalib_Channel_t *cal = &calib_data[ch_index] ;
    long mapped_val = 992 ;
    uint16_t filter_adc = raw_adc ;

    if (filter_adc < (cal->center - cal->deadband))
    {
        mapped_val = map(filter_adc, cal->min, cal->center - cal->deadband, 172, 992) ;
    }
    else if (filter_adc > (cal->center + cal->deadband))
    {
        mapped_val = map(filter_adc, cal->center + cal->deadband, cal->max, 992, 1811) ;
    }
    else
    {
        mapped_val = 992 ;
    }

    return clamp_channels(mapped_val) ;
}
