#ifndef __SYSTEM_HARDWARE_CONFIG_H
#define __SYSTEM_HARDWARE_CONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

/*
 * Hardware Integration Map (STM32F407VET6)
 * Project: jumpert-pro-TX
 * Purpose: Unified pin/peripheral reference for PCB integration
 */

/* UART: CRSF/ELRS half-duplex link (USART1) */
#define HW_UART_RF_INSTANCE                USART1
#define HW_UART_RF_BAUDRATE                400000U
#define HW_UART_RF_GPIO_PORT               GPIOA
#define HW_UART_RF_TX_PIN                  LL_GPIO_PIN_9   /* PA9  */
#define HW_UART_RF_RX_PIN                  LL_GPIO_PIN_10  /* PA10 */
#define HW_UART_RF_GPIO_AF                 LL_GPIO_AF_7

/* UART: Debug printf (USART2, PD5/PD6) */
#define HW_UART_DBG_INSTANCE               USART2
#define HW_UART_DBG_BAUDRATE               921600U
#define HW_UART_DBG_GPIO_PORT              GPIOD
#define HW_UART_DBG_TX_PIN                 LL_GPIO_PIN_5   /* PD5 */
#define HW_UART_DBG_RX_PIN                 LL_GPIO_PIN_6   /* PD6 */
#define HW_UART_DBG_GPIO_AF                LL_GPIO_AF_7

/* I2C: OLED bus (I2C1) */
#define HW_I2C_OLED_INSTANCE               I2C1
#define HW_I2C_OLED_SPEED                  400000U
#define HW_I2C_OLED_GPIO_PORT              GPIOB
#define HW_I2C_OLED_SCL_PIN                LL_GPIO_PIN_6   /* PB6 */
#define HW_I2C_OLED_SDA_PIN                LL_GPIO_PIN_7   /* PB7 */
#define HW_I2C_OLED_GPIO_AF                LL_GPIO_AF_4
#define HW_I2C_OLED_ADDR_8BIT              0x78U

/* 5-way keys (active low) */
#define HW_KEY_UP_PORT                     GPIOC
#define HW_KEY_UP_PIN                      GPIO_PIN_0      /* PC0 */
#define HW_KEY_DOWN_PORT                   GPIOC
#define HW_KEY_DOWN_PIN                    GPIO_PIN_1      /* PC1 */
#define HW_KEY_ENTER_PORT                  GPIOC
#define HW_KEY_ENTER_PIN                   GPIO_PIN_2      /* PC2 */
#define HW_KEY_BACK_PORT                   GPIOC
#define HW_KEY_BACK_PIN                    GPIO_PIN_3      /* PC3 */
#define HW_KEY_RETURN_PORT                 GPIOC
#define HW_KEY_RETURN_PIN                  GPIO_PIN_4      /* PC4 */

/* two stage lever*/
#define HW_TWO_LEVER_GPIO_PORT             GPIOA
#define HW_TWO_LEVER_GPIO_PIN              LL_GPIO_PIN_5  

/* three stage lever*/
#define HW_THREE_LEVER_GPIO_PORT             GPIOA
#define HW_THREE_LEVER_GPIO_PIN_1            LL_GPIO_PIN_6
#define HW_THREE_LEVER_GPIO_PIN_2            LL_GPIO_PIN_7  

/* ADC joystick: ADC1 CH1~CH4, trigger source TIM4_CH4 event
 * DMA scan order (rank): PA1 Throttle, PA2 Yaw, PA3 Pitch, PA4 Roll
 */
#define HW_ADC_JOY_INSTANCE                ADC1
#define HW_ADC_JOY_CH1                     LL_ADC_CHANNEL_1 /* PA1 Throttle */
#define HW_ADC_JOY_CH2                     LL_ADC_CHANNEL_2 /* PA2 Yaw */
#define HW_ADC_JOY_CH3                     LL_ADC_CHANNEL_3 /* PA3 Pitch */
#define HW_ADC_JOY_CH4                     LL_ADC_CHANNEL_4 /* PA4 Roll */
#define HW_ADC_JOY_RANK_COUNT              4U
#define HW_ADC_JOY_TRIGGER_TIM             TIM4
#define HW_ADC_JOY_TRIGGER_SOURCE          LL_ADC_REG_TRIG_EXT_TIM4_CH4
#define HW_ADC_JOY_THROTTLE_PORT           GPIOA
#define HW_ADC_JOY_THROTTLE_PIN            LL_GPIO_PIN_1
#define HW_ADC_JOY_YAW_PORT                GPIOA
#define HW_ADC_JOY_YAW_PIN                 LL_GPIO_PIN_2
#define HW_ADC_JOY_PITCH_PORT              GPIOA
#define HW_ADC_JOY_PITCH_PIN               LL_GPIO_PIN_3
#define HW_ADC_JOY_ROLL_PORT               GPIOA
#define HW_ADC_JOY_ROLL_PIN                LL_GPIO_PIN_4

/* CRSF RC channel index: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw */
#define HW_RC_CH_ROLL                      0U
#define HW_RC_CH_PITCH                     1U
#define HW_RC_CH_THROTTLE                  2U
#define HW_RC_CH_YAW                       3U

/* ADC scan rank index -> CRSF RC channel index */
#define HW_ADC_RANK_TO_RC_CH_TABLE         { HW_RC_CH_THROTTLE, HW_RC_CH_YAW, HW_RC_CH_PITCH, HW_RC_CH_ROLL }

/* Timers */
#define HW_TIM_RC_SCHED_INSTANCE           TIM3
#define HW_TIM_RC_SCHED_PSC                (168U - 1U)
#define HW_TIM_RC_SCHED_ARR                (2000U - 1U)

#define HW_TIM_ADC_TRIG_INSTANCE           TIM4
#define HW_TIM_ADC_TRIG_PSC                (84U - 1U)
#define HW_TIM_ADC_TRIG_ARR                (2000U - 1U)
#define HW_TIM_ADC_TRIG_CH                 LL_TIM_CHANNEL_CH4

/* DMA mapping */
#define HW_DMA_UART1_RX_INSTANCE           DMA2
#define HW_DMA_UART1_RX_STREAM             LL_DMA_STREAM_5
#define HW_DMA_UART1_RX_CHANNEL            LL_DMA_CHANNEL_4

#define HW_DMA_UART1_TX_INSTANCE           DMA2
#define HW_DMA_UART1_TX_STREAM             LL_DMA_STREAM_7
#define HW_DMA_UART1_TX_CHANNEL            LL_DMA_CHANNEL_4

#define HW_DMA_ADC1_INSTANCE               DMA2
#define HW_DMA_ADC1_STREAM                 LL_DMA_STREAM_4
#define HW_DMA_ADC1_CHANNEL                LL_DMA_CHANNEL_0

#define HW_DMA_OLED_I2C_TX_INSTANCE        DMA1
#define HW_DMA_OLED_I2C_TX_STREAM          LL_DMA_STREAM_6
#define HW_DMA_OLED_I2C_TX_CHANNEL         LL_DMA_CHANNEL_1

/* IRQ mapping */
#define HW_IRQ_UART1                       USART1_IRQn
#define HW_IRQ_DMA_UART1_TX                DMA2_Stream7_IRQn
#define HW_IRQ_DMA_ADC1                    DMA2_Stream4_IRQn
#define HW_IRQ_TIM3                        TIM3_IRQn

/* Core clock target */
#define HW_SYS_CORE_CLOCK_HZ               168000000UL

/* Internal flash: persist joystick calibration in sector 7 */
#define HW_FLASH_JOY_CALIB_SECTOR          7U
#define HW_FLASH_JOY_CALIB_ADDR            0x08060000UL

#endif
 