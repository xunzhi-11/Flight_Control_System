#ifndef __SYS_SOFTWARE_CONFIG_H
#define __SYS_SOFTWARE_CONFIG_H

#define COMPILER_BARRIER() __asm volatile("" ::: "memory")


/*============ DMA传输状态 ============*/
typedef enum {
    SDIO_DMA_IDLE = 0,
    SDIO_DMA_BUSY,
    SDIO_DMA_COMPLETE,
    SDIO_DMA_ERROR
} SDIO_DMA_State_e;

#define MOTOR_UNLOCKER_BOUNDARY_VALUE  1000U

/*task priority*/
#define SW_TASK_PRIORITY_PARSER        6U
#define SW_TASK_PRORITY_LOG            6U
#define SW_TASK_PRORITY_MOTOR_UNLOCKER 6U
#define SW_TASK_PRORITY_MODE_EDITOR    6U
#define SW_TASK_PRORITY_ALT_HOLD       6U
#define SW_TASK_PRORITY_POS_HOLD       6U
#define SW_TASK_PRIORITY_GPS           6U


/*task frequency*/
#define SW_TASK_PARSER_INTERVAL         2U
#define SW_TASK_GPS_INTERVAL            5U
#define SW_TASK_UNLOCKER_INTERVAL       300U
#define SW_TASK_MODE_EDITOR_INTERVAL    300U
#define SW_TASK_ALT_HOLD_INTERVAL       20U   /* 50 Hz */
#define SW_TASK_POS_HOLD_INTERVAL       50U   /* 20 Hz, GPS-limited */



#endif