#include "app_core/joy_calib_flow.h"
#include "app_core/rc_data_center.h"
#include "utils/joycal.h"
#include "app_task/app_task_joystick_update.h"
#include "OLED/OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

typedef enum {
    JOY_CALIB_STATE_CENTER = 0,
    JOY_CALIB_STATE_EXTREMES,
    JOY_CALIB_STATE_RESULT
} joy_calib_state_e ;

typedef enum {
    JOY_CALIB_RESULT_NONE = 0,
    JOY_CALIB_RESULT_SAVED,
    JOY_CALIB_RESULT_FAILED,
    JOY_CALIB_RESULT_CANCEL
} joy_calib_result_e ;

static bool s_active = false ;
static joy_calib_state_e s_state = JOY_CALIB_STATE_CENTER ;
static joy_calib_result_e s_result = JOY_CALIB_RESULT_NONE ;
static JoyCalib_Channel_t s_working[4] ;
static uint16_t s_result_timer_ms = 0 ;

static const uint16_t s_default_deadband[4] = {15, 15, 0, 15} ; /* Roll, Pitch, Throttle, Yaw */

static void joy_calib_reset_working(void)
{
    JoyCalib_Get_Current(s_working) ;
}

static void joy_calib_capture_center(void)
{
    uint32_t sum[4] = {0} ;

    for (uint8_t sample = 0 ; sample < 8U ; sample++)
    {
        uint16_t raw[4] ;

        Joystick_Get_Latest_Raw(raw) ;
        for (uint8_t adc_rank = 0 ; adc_rank < 4U ; adc_rank++)
        {
            sum[adc_rank] += raw[adc_rank] ;
        }

        vTaskDelay(pdMS_TO_TICKS(10)) ;
    }

    for (uint8_t adc_rank = 0 ; adc_rank < 4U ; adc_rank++)
    {
        uint8_t rc_ch = g_hw_adc_rank_to_rc_ch[adc_rank] ;

        s_working[rc_ch].center = (uint16_t)(sum[adc_rank] / 8U) ;
        s_working[rc_ch].min = s_working[rc_ch].center ;
        s_working[rc_ch].max = s_working[rc_ch].center ;
        s_working[rc_ch].deadband = s_default_deadband[rc_ch] ;
    }
}

static void joy_calib_update_extremes(void)
{
    uint16_t raw[4] ;

    Joystick_Get_Latest_Raw(raw) ;

    for (uint8_t adc_rank = 0 ; adc_rank < 4U ; adc_rank++)
    {
        uint8_t rc_ch = g_hw_adc_rank_to_rc_ch[adc_rank] ;

        if (raw[adc_rank] < s_working[rc_ch].min)
        {
            s_working[rc_ch].min = raw[adc_rank] ;
        }

        if (raw[adc_rank] > s_working[rc_ch].max)
        {
            s_working[rc_ch].max = raw[adc_rank] ;
        }
    }
}

static void joy_calib_finish(bool saved)
{
    s_state = JOY_CALIB_STATE_RESULT ;
    s_result = saved ? JOY_CALIB_RESULT_SAVED : JOY_CALIB_RESULT_FAILED ;
    s_result_timer_ms = 1200U ;
}

static void joy_calib_cancel(void)
{
    s_state = JOY_CALIB_STATE_RESULT ;
    s_result = JOY_CALIB_RESULT_CANCEL ;
    s_result_timer_ms = 800U ;
}

bool JoyCalib_Is_Active(void)
{
    return s_active ;
}

void JoyCalib_Start(void)
{
    s_active = true ;
    s_state = JOY_CALIB_STATE_CENTER ;
    s_result = JOY_CALIB_RESULT_NONE ;
    s_result_timer_ms = 0 ;
    joy_calib_reset_working() ;
}

void JoyCalib_Process(Input_Action_e action)
{
    if (!s_active)
    {
        return ;
    }

    if (s_state == JOY_CALIB_STATE_RESULT)
    {
        return ;
    }

    switch (action)
    {
        case ACTION_BACK:
            joy_calib_cancel() ;
            break ;

        case ACTION_ENTER:
            if (s_state == JOY_CALIB_STATE_CENTER)
            {
                joy_calib_capture_center() ;
                s_state = JOY_CALIB_STATE_EXTREMES ;
            }
            else if (s_state == JOY_CALIB_STATE_EXTREMES)
            {
                if (JoyCalib_Save(s_working))
                {
                    joy_calib_finish(true) ;
                }
                else
                {
                    joy_calib_finish(false) ;
                }
            }
            break ;

        default:
            break ;
    }
}

void JoyCalib_Tick(void)
{
    if (!s_active)
    {
        return ;
    }

    if (s_state == JOY_CALIB_STATE_EXTREMES)
    {
        joy_calib_update_extremes() ;
    }

    if (s_state == JOY_CALIB_STATE_RESULT)
    {
        if (s_result_timer_ms > 50U)
        {
            s_result_timer_ms = (uint16_t)(s_result_timer_ms - 50U) ;
        }
        else
        {
            s_active = false ;
            s_state = JOY_CALIB_STATE_CENTER ;
            s_result = JOY_CALIB_RESULT_NONE ;
            s_result_timer_ms = 0 ;
        }
    }
}

void JoyCalib_Render(void)
{
    OLED_Clear() ;

    if (s_state == JOY_CALIB_STATE_CENTER)
    {
        OLED_ShowString(1, 1, "JOY CALIB 1/2") ;
        OLED_ShowString(2, 1, "Center Sticks") ;
        OLED_ShowString(3, 1, "ENTER=Next") ;
        OLED_ShowString(4, 1, "BACK=Cancel") ;
    }
    else if (s_state == JOY_CALIB_STATE_EXTREMES)
    {
        OLED_ShowString(1, 1, "JOY CALIB 2/2") ;
        OLED_ShowString(2, 1, "Move All Axes") ;
        OLED_ShowString(3, 1, "ENTER=Save") ;
        OLED_ShowString(4, 1, "BACK=Cancel") ;
    }
    else
    {
        switch (s_result)
        {
            case JOY_CALIB_RESULT_SAVED:
                OLED_ShowString(2, 1, "Calib Saved") ;
                break ;
            case JOY_CALIB_RESULT_FAILED:
                OLED_ShowString(2, 1, "Save Failed") ;
                break ;
            case JOY_CALIB_RESULT_CANCEL:
                OLED_ShowString(2, 1, "Cancelled") ;
                break ;
            default:
                break ;
        }
    }

    OLED_Update() ;
}
