#include "app_task/app_task_joystick_update.h"
#include "app_core/rc_data_center.h"
#include "system_hardware_config.h"
#include "utils/joycal.h"
#include "dma/ll_dma.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"


typedef struct 
{
    uint16_t adc_raw_buf[8] ; 
    uint16_t ch_pwm[4] ; 
} Joystick_Buffer_d ;


TaskHandle_t     Joystick_Data_Update = NULL ; 
Joystick_Buffer_d joystick = {0} ; 
static volatile uint8_t s_latest_adc_offset = 0 ;

uint16_t* joystick_get_rawbuf_addr(void)
{
    return joystick.adc_raw_buf ; 
}

void Joystick_Get_Latest_Raw(uint16_t raw[4])
{
    uint8_t offset = s_latest_adc_offset ;

    for (uint8_t i = 0 ; i < 4U ; i++)
    {
        raw[i] = joystick.adc_raw_buf[offset + i] ;
    }
}

void DMA_FOR_ADC1_Callback(uint8_t is_full) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE ;

    if (Joystick_Data_Update != NULL) 
    {
        xTaskNotifyFromISR(Joystick_Data_Update, is_full, eSetValueWithOverwrite, &xHigherPriorityTaskWoken) ;
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken) ;
}

static void Joystick_Update_Task(void* pdParameters)
{
    DMA_ADC_RegisterCallback(DMA_FOR_ADC1_Callback) ; 

    uint32_t buffer_event = 0 ;
    while (1)
    {
        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &buffer_event, portMAX_DELAY) == pdTRUE)
        {
            uint8_t offset = (uint8_t)(buffer_event * 4U) ;
            s_latest_adc_offset = offset ;

            for (int adc_rank = 0 ; adc_rank < 4 ; adc_rank++)
            {
                uint8_t rc_ch = g_hw_adc_rank_to_rc_ch[adc_rank] ;
                uint16_t raw_val = joystick.adc_raw_buf[offset + adc_rank] ;
                joystick.ch_pwm[rc_ch] = Process_Joystick_Channel(rc_ch, raw_val) ;
            }
            joystick_channels_update(joystick.ch_pwm) ;
        }
    }
}

void Task_Joystick_Init(void)
{
    xTaskCreate(Joystick_Update_Task , "Joystick_Update_Task" , 256 , NULL , 5 , &Joystick_Data_Update) ; 
}
