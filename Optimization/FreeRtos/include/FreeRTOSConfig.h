#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 * 针对 STM32F407VET6 (Cortex-M4F, 168MHz, 192KB RAM) 优化配置
 *---------------------------------------------------------*/

// 内核配置 --------------------------------------------------
#define configUSE_PREEMPTION                     1      // 使用抢占式调度器
#define configUSE_TICKLESS_IDLE                  0      // 不使用 tickless 模式 (根据需求开启)
#define configCPU_CLOCK_HZ                       ( 168000000UL ) // ***核心修改：STM32F407典型频率 168MHz***
#define configTICK_RATE_HZ                       ( 1000 )       // 系统时钟频率 1kHz (1ms)
#define configMAX_PRIORITIES                     ( 10 )         // 最大任务优先级数（F4资源多，可适当增加）
#define configMINIMAL_STACK_SIZE                 ( 128 )        // 空闲任务最小栈大小 (128 words, 512 bytes)
#define configMAX_TASK_NAME_LEN                  ( 16 )         // 任务名称最大长度
#define configUSE_16_BIT_TICKS                   0              // 使用32位时钟计数器
#define configIDLE_SHOULD_YIELD                  1              // 空闲任务中创建的任务会立即抢占空闲任务

// 内存配置 --------------------------------------------------
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 64 * 1024 ) ) // ***核心修改：F4资源多，增大堆大小到 64KB***
#define configAPPLICATION_ALLOCATED_HEAP         0      // 堆内存由FreeRTOS管理
#define configSUPPORT_STATIC_ALLOCATION          0      // 不使用静态内存分配
#define configSUPPORT_DYNAMIC_ALLOCATION         1      // 使用动态内存分配

// 钩子函数配置 ----------------------------------------------
#define configUSE_IDLE_HOOK                      0      // 不使用空闲任务钩子函数
#define configUSE_TICK_HOOK                      0      // 不使用时钟滴答钩子函数
#define configUSE_MALLOC_FAILED_HOOK             0      // ***建议开启：使用内存分配失败钩子函数进行调试***

// 运行统计配置 --------------------------------------------
#define configGENERATE_RUN_TIME_STATS            0      // 不生成运行时间统计信息
#define configUSE_TRACE_FACILITY                 0      // 不使用跟踪功能 (调试时可开启)
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

// 协程配置 --------------------------------------------------
#define configUSE_CO_ROUTINES                    0      // 不使用协程

// 软件定时器配置 --------------------------------------------
#define configUSE_TIMERS                         1      // 使用软件定时器
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 2 ) // 定时器服务任务优先级，设为高优先级
#define configTIMER_QUEUE_LENGTH                 ( 10 )         // 定时器命令队列长度
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 4 ) // ***增大定时器任务栈深度***

// 任务通知配置 ----------------------------------------------
#define configUSE_TASK_NOTIFICATIONS             1      // 使用任务通知功能

// 同步机制配置 ----------------------------------------------
#define configUSE_MUTEXES                        1      // 使用互斥量
#define configUSE_RECURSIVE_MUTEXES              1      // 使用递归互斥量
#define configUSE_COUNTING_SEMAPHORES            1      // 使用计数信号量
#define configUSE_QUEUE_SETS                     0      // 不使用队列集

// API 函数包含配置 
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1      
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_pcTaskGetTaskName                1

#define configPRIO_BITS                          4      // STM32F4xx 系列通常配置为 4 位优先级 (0-15)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15     // 最低中断优先级 (数字越大，优先级越低)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5 

// 计算实际的中断优先级值 (用于 FreeRTOS 内部)
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

//  断言配置 
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

//  硬件特性配置 (针对 STM32F407) --------------------------------------------
#define configENABLE_FPU                         1      // 启用浮点单元 (FPU) 支持
#define configENABLE_MPU                         0      // 不使用内存保护单元

//  中断向量重定义 
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

// 1临界区保护 (可选) --------------------------------------------
// 如果需要兼容旧版/其他处理器架构，可以定义这些宏
//#define portCONFIGURE_APPROPRIATE_INTERRUPT_PRIORITY
//#define portSET_INTERRUPT_MASK_FROM_ISR()
//#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)

#endif /* FREERTOS_CONFIG_H */
