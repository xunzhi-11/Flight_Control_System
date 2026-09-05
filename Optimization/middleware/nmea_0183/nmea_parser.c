#include "nmea_0183/nmea_parser.h"
#include "nmea_0183/nmea_protocol.h"
#include "app_core/app_system_data_center.h"
#include "stdbool.h"

typedef enum
{
    NMEA_STATE_SYNC = 0,
    NMEA_STATE_PAYLOAD,
    NMEA_STATE_CHECKSUM_H,
    NMEA_STATE_CHECKSUM_L,
    NMEA_STATE_CR,
} NMEA_State_t;

typedef struct
{
    NMEA_State_t state;
    char buffer[NMEA_SENTENCE_MAX_LEN];
    uint8_t idx;
    uint8_t calc_checksum;
    uint8_t recv_checksum;
} NMEA_Parser_Context_t;

static NMEA_Parser_Context_t s_nmea_ctx = {0};

static uint8_t nmea_hex_nibble(char c)
{
    if (c >= '0' && c <= '9')
    {
        return (uint8_t)(c - '0');
    }
    if (c >= 'A' && c <= 'F')
    {
        return (uint8_t)(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f')
    {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0xFF;
}

static double nmea_atof(const char *s)
{
    double value = 0.0;
    double frac = 0.0;
    double div = 1.0;
    uint8_t neg = 0U;

    if (s == NULL || s[0] == '\0')
    {
        return 0.0;
    }

    if (*s == '-')
    {
        neg = 1U;
        s++;
    }

    while (*s >= '0' && *s <= '9')
    {
        value = value * 10.0 + (double)(*s - '0');
        s++;
    }

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            frac = frac * 10.0 + (double)(*s - '0');
            div *= 10.0;
            s++;
        }
    }

    value += frac / div;
    return neg ? -value : value;
}

static uint8_t nmea_field_len(const char *field)
{
    uint8_t len = 0U;

    if (field == NULL)
    {
        return 0U;
    }

    while (field[len] != '\0')
    {
        len++;
    }

    return len;
}

static uint8_t nmea_split_fields(char *sentence, char *fields[], uint8_t max_fields)
{
    uint8_t count = 0U;

    if (sentence == NULL || max_fields == 0U)
    {
        return 0U;
    }

    fields[count++] = sentence;

    for (char *p = sentence; *p != '\0' && count < max_fields; p++)
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    return count;
}

static bool nmea_sentence_is(const char *field0, const char *tag)
{
    uint8_t f0_len;
    uint8_t tag_len;

    if (field0 == NULL || tag == NULL)
    {
        return false;
    }

    f0_len = nmea_field_len(field0);
    tag_len = nmea_field_len(tag);

    if (f0_len < tag_len)
    {
        return false;
    }

    for (uint8_t i = 0U; i < tag_len; i++)
    {
        if (field0[f0_len - tag_len + i] != tag[i])
        {
            return false;
        }
    }

    return true;
}

static void nmea_parse_utc_time(const char *field, atgm336h_data_t *out)
{
    uint8_t len;

    if (field == NULL || out == NULL || field[0] == '\0')
    {
        return;
    }

    len = nmea_field_len(field);
    if (len < 6U)
    {
        return;
    }

    out->hour = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
    out->minute = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
    out->second = (uint8_t)((field[4] - '0') * 10 + (field[5] - '0'));
    out->msec = 0U;

    if (len > 7U && field[6] == '.')
    {
        uint16_t frac = 0U;
        uint16_t mul = 100U;

        for (uint8_t i = 7U; i < len && mul > 0U; i++)
        {
            if (field[i] < '0' || field[i] > '9')
            {
                break;
            }
            frac += (uint16_t)(field[i] - '0') * mul;
            mul = (uint16_t)(mul / 10U);
        }
        out->msec = frac;
    }
}

static void nmea_parse_utc_date(const char *field, atgm336h_data_t *out)
{
    if (field == NULL || out == NULL || field[0] == '\0')
    {
        return;
    }

    if (nmea_field_len(field) < 6U)
    {
        return;
    }

    out->day = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
    out->month = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
    out->year = (uint16_t)((field[4] - '0') * 10 + (field[5] - '0') + 2000U);
}

static double nmea_ddmm_to_deg(const char *field, char hem)
{
    double raw;
    uint32_t deg;
    double minutes;

    if (field == NULL || field[0] == '\0')
    {
        return 0.0;
    }

    raw = nmea_atof(field);
    deg = (uint32_t)(raw / 100.0);
    minutes = raw - (double)deg * 100.0;

    raw = (double)deg + minutes / 60.0;
    if (hem == 'S' || hem == 'W')
    {
        raw = -raw;
    }

    return raw;
}

static void nmea_handle_gga(char *fields[], uint8_t field_count)
{
    atgm336h_data_t partial = {0};

    if (field_count < 10U)
    {
        return;
    }

    if (fields[2][0] == '\0' || fields[4][0] == '\0')
    {
        return;
    }

    nmea_parse_utc_time(fields[1], &partial);
    partial.latitude = nmea_ddmm_to_deg(fields[2], fields[3][0]);
    partial.longitude = nmea_ddmm_to_deg(fields[4], fields[5][0]);
    partial.fix_quality = (uint8_t)nmea_atof(fields[6]);
    partial.satellites = (uint8_t)nmea_atof(fields[7]);
    partial.hdop = (float)nmea_atof(fields[8]);
    partial.altitude = (float)nmea_atof(fields[9]);

    gps_apply_gga(&partial);
}

static void nmea_handle_rmc(char *fields[], uint8_t field_count)
{
    atgm336h_data_t partial = {0};

    if (field_count < 10U)
    {
        return;
    }

    nmea_parse_utc_time(fields[1], &partial);
    partial.is_valid = (fields[2][0] == 'A');

    if (fields[3][0] != '\0' && fields[5][0] != '\0')
    {
        partial.latitude = nmea_ddmm_to_deg(fields[3], fields[4][0]);
        partial.longitude = nmea_ddmm_to_deg(fields[5], fields[6][0]);
    }

    if (fields[7][0] != '\0')
    {
        partial.speed_kmh = (float)(nmea_atof(fields[7]) * 1.852);
    }

    if (fields[8][0] != '\0')
    {
        partial.course = (float)nmea_atof(fields[8]);
    }

    nmea_parse_utc_date(fields[9], &partial);
    gps_apply_rmc(&partial);
}

static void nmea_dispatch_sentence(void)
{
    char *fields[NMEA_MAX_FIELDS];
    uint8_t field_count;

    s_nmea_ctx.buffer[s_nmea_ctx.idx] = '\0';
    field_count = nmea_split_fields(s_nmea_ctx.buffer, fields, NMEA_MAX_FIELDS);

    if (field_count == 0U)
    {
        return;
    }

    if (nmea_sentence_is(fields[0], "GGA"))
    {
        nmea_handle_gga(fields, field_count);
    }
    else if (nmea_sentence_is(fields[0], "RMC"))
    {
        nmea_handle_rmc(fields, field_count);
    }
}

static void nmea_reset_parser(void)
{
    s_nmea_ctx.state = NMEA_STATE_SYNC;
    s_nmea_ctx.idx = 0U;
    s_nmea_ctx.calc_checksum = 0U;
    s_nmea_ctx.recv_checksum = 0U;
}

static void nmea_finish_sentence(void)
{
    if (s_nmea_ctx.calc_checksum == s_nmea_ctx.recv_checksum)
    {
        nmea_dispatch_sentence();
    }

    nmea_reset_parser();
}

void NMEA_Parser_Byte(uint8_t byte)
{
    switch (s_nmea_ctx.state)
    {
    case NMEA_STATE_SYNC:
        if (byte == '$')
        {
            s_nmea_ctx.state = NMEA_STATE_PAYLOAD;
            s_nmea_ctx.idx = 0U;
            s_nmea_ctx.calc_checksum = 0U;
        }
        break;

    case NMEA_STATE_PAYLOAD:
        if (byte == '*')
        {
            s_nmea_ctx.state = NMEA_STATE_CHECKSUM_H;
            s_nmea_ctx.buffer[s_nmea_ctx.idx] = '\0';
            break;
        }

        if (s_nmea_ctx.idx >= (NMEA_SENTENCE_MAX_LEN - 1U))
        {
            nmea_reset_parser();
            break;
        }

        s_nmea_ctx.buffer[s_nmea_ctx.idx++] = (char)byte;
        s_nmea_ctx.calc_checksum ^= byte;
        break;

    case NMEA_STATE_CHECKSUM_H:
    {
        uint8_t hi = nmea_hex_nibble((char)byte);
        if (hi == 0xFF)
        {
            nmea_reset_parser();
            break;
        }
        s_nmea_ctx.recv_checksum = (uint8_t)(hi << 4);
        s_nmea_ctx.state = NMEA_STATE_CHECKSUM_L;
        break;
    }

    case NMEA_STATE_CHECKSUM_L:
    {
        uint8_t lo = nmea_hex_nibble((char)byte);
        if (lo == 0xFF)
        {
            nmea_reset_parser();
            break;
        }
        s_nmea_ctx.recv_checksum |= lo;
        s_nmea_ctx.state = NMEA_STATE_CR;
        break;
    }

    case NMEA_STATE_CR:
        if (byte == '\r')
        {
            break;
        }
        if (byte == '\n')
        {
            nmea_finish_sentence();
            break;
        }
        if (byte == '$')
        {
            nmea_finish_sentence();
            s_nmea_ctx.state = NMEA_STATE_PAYLOAD;
            s_nmea_ctx.idx = 0U;
            s_nmea_ctx.calc_checksum = 0U;
        }
        else
        {
            nmea_reset_parser();
        }
        break;

    default:
        nmea_reset_parser();
        break;
    }
}
