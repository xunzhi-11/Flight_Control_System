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
//bsp
#include "oled/OLED.h"
#include "bmm150/bmm150.h"
#include "icm42688/icm42688.h"
#include "bmp390l/bmp390l.h"
#include "sd_card/sd_card.h"
//mcu driver
#include "dma/ll_dma_spi.h"
#include "dma/ll_dma_iic.h"
#include "dma/ll_dma_uart.h"
#include "dma/ll_dma_tim.h"
#include "dma/ll_dma_sdio.h"
#include "nvic/ll_nvic.h"
#include "exti/ll_exti.h"
#include "spi/ll_spi.h"
#include "iic/ll_i2c.h"
#include "gpio/ll_gpio_bmm150.h"
#include "gpio/ll_gpio_icm42688.h"
#include "gpio/ll_gpio_bmp390l.h"
#include "gpio/ll_gpio_uart.h"
#include "gpio/ll_gpio_dshot.h"
#include "gpio/ll_gpio_sdio.h"
#include "tim/tim5.h"
#include "tim/tim1.h"
#include "uart/ll_uart.h"
#include "sdio/ll_sdio.h"

//system
#include "system_clock/System_Clock.h"
#include "spi_device.h"
#include "data_structure_types.h"
//software
#include "system_delay/delay.h"
#include "callback_center.h"
#include "app_core/app_sensor_data_center.h"
#include "app_core/app_system_data_center.h"
#include "app_core/app_flight_mode.h"
#include "crsf/CRSF_Parser.h"
#include "utils/madgwick/Madgwick_Fusion.h"

//freertos task
#include "app_task/app_task_parser.h"
#include "app_task/app_task_gps.h"
#include "app_core/app_system_data_center.h"
#include "app_task/app_task_log.h"
#include "app_task/app_task_ahrs.h"
#include "app_task/app_task_motor_unlocker.h"
#include "app_task/app_task_mode_editor.h"
#include "app_task/app_task_alt_hold.h"
#include "app_task/app_task_pos_hold.h"
#include "bmp390l_drdy_sim.h"

#include "utils/mixer/mixer.h"
#include "utils/dwt/dwt_profiler.h"
#include "vib_log/vib_log.h"
#include "vib_log/vib_log_config.h"
#include "pid_trace/pid_trace.h"
#include "pid_trace/pid_trace_config.h"

void my_yield(void) 
{
    vTaskDelay(pdMS_TO_TICKS(1)); 
}

void System_SoftWare_Init(void)
{
    SystemClock_Config() ; 
    System_Set1msTick(168000000);
    DWT_Init(); 
    delay_Init() ;
    BMM150_Register_Delay(delay_ms) ; 
    icm42688_register_delay(delay_ms) ; 
    BMP390l_Register_Delay(delay_ms) ; 
    SDIO_Register_Delay(delay_ms) ; 
    SD_Card_Delay_us_Register(delay_us) ;
    SD_Card_Delay_ms_Register(delay_ms) ; 
    SD_Register_OS_Hooks((sd_get_tick_f)xTaskGetTickCount, my_yield);
    FlightMode_Init();
    GyroCalibrator_Init(gyro_claib_get_instance());
    AccCalibrator_Init(acc_calib_get_instance());
    BaroCalibrator_Init(baro_calib_get_instance());
    IMU_Filter_Init();
    BMM150_Manual_Calibration_AllSystem() ; 
    MadgwickFusion_Init(NULL) ; 
    MadgwickFusion_SetBeta(0.03f);
    MadgwickFusion_SetZeta(0.0001f); 
}

void System_HardWare_Init(void)
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    //test
    OLED_Init() ; 

    //tim
    Time5_Stamp_Init() ; 
    tim1_dshot_init() ; 

    //gpio
    gpio_bmm150_init();        
    gpio_spi1_Init();           
    gpio_bmm150_drdy_init();   
    
    gpio_icm42688_init() ; 
    gpio_spi2_init() ; 
    gpio_icm42688_drdy_init() ; 

    gpio_i2c2_init() ; 
    gpio_bmp390l_drdy_init() ; 

    gpio_uart_init() ; 

    gpio_dshot_init() ; 

    gpio_sdio_init() ; 
    //spi
    spi_bmm150_init() ;  
    spi_icm42688_init() ;      
    
    //iic
    iic_bmp390l_init() ; 

    //uart
    UART_Config_Init() ; 
    VibLog_Init();
    PidTrace_Init();

    //sdio
    sdio_sdcard_init() ; 
   
    //dma
    dma_to_spi1_rx_init();    
    dma_to_spi2_rx_init() ; 
    dma_to_iic2_rx_init() ; 
    dma_for_uart2_init() ; 
    dma_for_uart3_init() ;
    dma_to_tim1_init() ; 
    dma_to_sdio_init() ;

    //exti
    exti_bmm150_drdy_config();  
    exti_icm42688_drdy_config() ; 
    exti_bmp390l_drdy_config() ; 

    //nvic
    nvic_dma_to_spi1_config() ; 
    nvic_dma_to_spi2_config() ; 
    nvic_dma_to_iic2_config() ; 
    nvic_dma_to_uart2_config() ; 
    nvic_dma_to_tim1_config() ; 
    nvic_dma_to_sdio_config() ; 
    nvic_ahrs_task_config() ; 

    //  软件路由配置
    CallbackCenter_Init();      // 挂载所有回调钩子
    gps_init();
    
    // 业务层硬件配置 
    bmm150_init() ; 
    icm42688_init() ; 
   
    bmp390l_init() ;  

    /* SD 协议初始化推迟到 disk_initialize / log_task，避免过早 init 后卡状态失效 */

    // 开启触发源头
    EXTI_BMM150_Start();       
    EXTI_ICM42688_Start() ; 
    EXTI_BMP390L_Start() ; 

    /* 临时：PC1 模拟 DRDY，短接 PC1 -> PB5 */
    bmp390l_drdy_sim_hw_init();
}

void task_test(void* pvParameters)
{
    uint32_t last_display_time = TIM5_GetStamp(); 
    MIXER_INPUT *mix_test;
    const MadgwickFusion_State_t* fusion_state = MadgwickFusion_GetStatePtr(); 
    crsf_channels_t channel = {0} ; 
    IMU_Scaled_t *imu_test = imu_update();
    while(1)
    {
        PidTrace_Service();

        if (TIM5_GetStamp() - last_display_time >= 500000) 
        {
            last_display_time = TIM5_GetStamp() ;

            CRSF_ReadChannels_Control(&channel) ; 
            printf("RF: %d , %d , %d , %d , %d , %d\r\n" , channel.ch0 , 
                                        channel.ch1 , channel.ch2 , channel.ch3 , channel.ch4 , channel.ch5) ; 

            mix_test = test() ; 
            printf ("mixer : %.2f , %.2f , %.2f , %.2f\r\n" , mix_test->pitch , mix_test->roll , mix_test->yaw , mix_test->throttle) ; 

            printf("A: %.2f , %.2f , %.2f \r\nG: %.2f , %.2f , %.2f\r\n " , 
                                imu_test->ax , imu_test->ay ,imu_test->az , imu_test->gx , imu_test->gy , imu_test->gz ) ; 

            printf("fusion: %.2f , %.2f , %.2f  \r\n" , fusion_state->euler.pitch , 
                                    fusion_state->euler.roll , fusion_state->euler.yaw ) ; 

#if PIDTRACE_ENABLE
            printf("pidtrace: rec=%u frames=%lu drop=%lu dumping=%u pa0=%u ch2=%u last_dump=%lu\r\n",
                   (unsigned)PidTrace_IsRecording(),
                   (unsigned long)PidTrace_GetFrameCount(),
                   (unsigned long)PidTrace_GetDropCount(),
                   (unsigned)PidTrace_IsDumping(),
                   (unsigned)PidTrace_IsTriggerPressed(),
                   (unsigned)channel.ch2,
                   (unsigned long)PidTrace_GetLastDumpFrames());
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(10)) ; 
    }
}


void App_Start(void)
{ 
    xTaskCreate(task_test , "task_test" , 512 , NULL , 5 , NULL) ; 
    task_parser_init() ; 
    task_gps_init();
    task_ahrs_init() ; 
    // task_log_init() ; 
    task_motor_unlocker_init() ; 
    task_mode_editor_init() ; 
    task_alt_hold_init() ;
    task_pos_hold_init() ;
    bmp390l_drdy_sim_task_start();
    vTaskStartScheduler() ; 
}