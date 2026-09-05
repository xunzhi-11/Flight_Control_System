#include "System_Application.h"
//FreeRTOS
#include "FreeRTOS.h"
#include "Task.h"
#include "Queue.h"
#include "timers.h"
#include "event_groups.h"
//C Library
#include "stdio.h"
#include "string.h"
//hardware

#include "OLED/OLED.h"
#include "gpio/ll_gpio.h"
#include "uart/ll_uart.h"
#include "dma/ll_dma.h"
#include "nvic/ll_nvic.h"
#include "tim/ll_tim3.h"
#include "adc/ll_adc.h"
#include "iic/ll_iic.h"
//software
#include "System_Delay/delay.h"
#include "System_Clock/System_Clock.h"
#include "app_task/app_task_parser.h"
#include "app_task/app_task_joystick_update.h"
#include "app_task/app_task_motor_unlocker.h"
#include "app_task/app_task_mode_editor.h"
#include "tim/ll_tim4.h"
#include "app_task/app_task_telemetry.h"
#include "app_task/app_task_input_scan.h"
#include "app_task/app_task_manager.h"
#include "app_task/app_task_UI.h"
#include "app_core/system_state.h"
#include "utils/joycal.h"

void System_SoftWare_Init(void)
{
    SystemClock_Config() ; 
    System_Set1msTick(168000000) ; 
    delay_Init() ;
}

void System_HardWare_Init(void)
{
    //GPIO
    UART_GPIO_Init() ; 
    OLED_GPIO_Init() ; 
    five_way_key_GPIO_Init() ; 
    JoyStick_GPIO_Init() ; 
    TWO_Stage_Lever_GPIO_Init() ; 
    THREE_Stage_Lever_GPIO_Init() ; 
    //ADC
    ADC_FOR_Joystick_Init() ; 
    //UART
    UART_Config_Init() ; 
    //DMA
    DMA_FOR_UART1_Init() ; 
    DMA_FOR_ADC1_Init() ; 
    DMA_FOR_OLED_Init() ; 
    //NVIC
    NVIC_UART1_Config() ; 
    NVIC_TIM3_Config() ; 
    NVIC_ADC1_Config() ;
    //TIM
    TIM3_Config() ; 
    TIM4_Config() ; 
    //IIC
    OLED_IIC_Init() ;   
    //BSP
    OLED_Init() ;
}

void App_Start(void)
{ 
    JoyCalib_Load() ;
    STATE_Init() ;   
    Task_Parser_Init() ; 
    Task_Joystick_Init() ; 
    Task_Tele_Init() ; 
    Task_InputScan_Init() ;
    Task_Manager_Init() ;
    Task_UI_Init() ; 
    task_motor_unlocker_init() ;
    task_mode_editor_init() ;
    vTaskStartScheduler() ; 
}