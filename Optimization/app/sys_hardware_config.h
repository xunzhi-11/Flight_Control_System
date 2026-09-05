#ifndef __SYS_HARDWARE_CONFIG_H
#define __SYS_HARDWARE_CONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"


#define COMPILER_BARRIER() __asm volatile("" ::: "memory")

/*
 * Hardware Integration Map (STM32F407 Flight Controller)
 * Project: Flight Controller 
 * Purpose: Unified pin/peripheral reference for PCB integration
 */ 

/* SPI1: BMM150 Magnetometer */
#define HW_SPI_BMM150_INSTANCE             SPI1
#define HW_SPI_BMM150_GPIO_PORT            GPIOA
#define HW_SPI_BMM150_SCK_PIN              LL_GPIO_PIN_5   /* PA5  */
#define HW_SPI_BMM150_MISO_PIN             LL_GPIO_PIN_6   /* PA6  */
#define HW_SPI_BMM150_MOSI_PIN             LL_GPIO_PIN_7   /* PA7  */
#define HW_SPI_BMM150_GPIO_AF              LL_GPIO_AF_5
#define HW_GPIO_BMM150_PS_PORT             GPIOA
#define HW_GPIO_BMM150_PS_PIN              LL_GPIO_PIN_4   /* PA4  */
#define HW_GPIO_BMM150_CS_PORT             GPIOA
#define HW_GPIO_BMM150_CS_PIN              LL_GPIO_PIN_8   /* PA8  */
#define HW_GPIO_BMM150_DRDY_PORT           GPIOA
#define HW_GPIO_BMM150_DRDY_PIN            LL_GPIO_PIN_1   /* PA1  */
#define HW_EXTI_BMM150_LINE                LL_EXTI_LINE_1

/* SPI2: ICM42688 IMU */ 
#define HW_SPI_ICM42688_INSTANCE           SPI2
#define HW_SPI_ICM42688_GPIO_PORT          GPIOB
#define HW_SPI_ICM42688_SCK_PIN            LL_GPIO_PIN_13  /* PB13 */
#define HW_SPI_ICM42688_MISO_PIN           LL_GPIO_PIN_14  /* PB14 */
#define HW_SPI_ICM42688_MOSI_PIN           LL_GPIO_PIN_15  /* PB15 */
#define HW_SPI_ICM42688_GPIO_AF            LL_GPIO_AF_5
#define HW_GPIO_ICM42688_CS_PORT           GPIOB
#define HW_GPIO_ICM42688_CS_PIN            LL_GPIO_PIN_12  /* PB12 */
#define HW_GPIO_ICM42688_DRDY_PORT         GPIOC
#define HW_GPIO_ICM42688_DRDY_PIN          LL_GPIO_PIN_0   /* PC0  */
#define HW_EXTI_ICM42688_LINE              LL_EXTI_LINE_0

/* exti_line_2 : ahrs_task */
#define HW_EXTIAHRS_TASK_LINE              LL_EXTI_LINE_2

/* I2C2: BMP390L Barometer */ 
#define HW_I2C_BMP390L_INSTANCE            I2C2
#define HW_I2C_BMP390L_GPIO_PORT           GPIOB
#define HW_I2C_BMP390L_SCL_PIN             LL_GPIO_PIN_10  /* PB10 */
#define HW_I2C_BMP390L_SDA_PIN             LL_GPIO_PIN_11  /* PB11 */
#define HW_I2C_BMP390L_GPIO_AF             LL_GPIO_AF_4
#define HW_GPIO_BMP390L_DRDY_PORT          GPIOB
#define HW_GPIO_BMP390L_DRDY_PIN           LL_GPIO_PIN_5   /* PB5  */
#define HW_EXTI_BMP390L_LINE               LL_EXTI_LINE_5
#define HW_I2C_BMP390L_SPEED               400000U
#define HW_I2C_BMP390L_ADDR_8BIT           0xEE

/* I2C1: OLED bus */
#define HW_I2C_OLED_INSTANCE               I2C1
#define HW_I2C_OLED_GPIO_PORT              GPIOB
#define HW_I2C_OLED_SCL_PIN                LL_GPIO_PIN_8   /* PB8  */
#define HW_I2C_OLED_SDA_PIN                LL_GPIO_PIN_9   /* PB9  */
#define HW_I2C_OLED_GPIO_AF                LL_GPIO_AF_4
#define HW_I2C_OLED_ADDR_8BIT              0x78U

/* SDIO: SD Card (Occupying previous SPI3 pins) */
#define HW_SDIO_INSTANCE                   SDIO
#define HW_SDIO_D0_PORT                    GPIOC
#define HW_SDIO_D0_PIN                     LL_GPIO_PIN_8   /* PC8  */
#define HW_SDIO_D1_PORT                    GPIOC
#define HW_SDIO_D1_PIN                     LL_GPIO_PIN_9   /* PC9  */
#define HW_SDIO_D2_PORT                    GPIOC
#define HW_SDIO_D2_PIN                     LL_GPIO_PIN_10  /* PC10 */
#define HW_SDIO_D3_PORT                    GPIOC
#define HW_SDIO_D3_PIN                     LL_GPIO_PIN_11  /* PC11 */
#define HW_SDIO_CK_PORT                    GPIOC
#define HW_SDIO_CK_PIN                     LL_GPIO_PIN_12  /* PC12 */
#define HW_SDIO_CMD_PORT                   GPIOD
#define HW_SDIO_CMD_PIN                    LL_GPIO_PIN_2   /* PD2  */
#define HW_SDIO_GPIO_AF                    LL_GPIO_AF_12
// 时钟分频配置1
#define SDIO_INIT_CLK_DIV                  118U    // 48MHz/(118+2) ≈ 400kHz
#define SDIO_TRANSFER_CLK_DIV              2U      // 48MHz/(2+2) = 12MHz，提高读写可靠性
// 数据块大小 
#define SDIO_BLOCK_SIZE                    512U
#define SDIO_BLOCK_SIZE_SHIFT              9U      // 2^9 = 512
//超时值 
#define SDIO_CMD_TIMEOUT                   0x00FFFFFFU
#define SDIO_DATA_TIMEOUT                  0x0FFFFFFFU
#define SDIO_DATATIMER_VALUE               0xFFFFFFFFU
//状态标志清除
#define SDIO_ICR_ALL_FLAGS                 0x7FFU



/* UART3: ATGM336H GPS */
#define HW_UART_GPS_INSTANCE               USART3
#define HW_UART_GPS_GPIO_PORT              GPIOD
#define HW_UART_GPS_BAUDRATE               115200U
#define HW_UART_GPS_TX_PIN                 LL_GPIO_PIN_8   /* PD8  */
#define HW_UART_GPS_RX_PIN                 LL_GPIO_PIN_9   /* PD9  */
#define HW_GPIO_GPS_PPS_PORT               GPIOD
#define HW_GPIO_GPS_PPS_PIN                LL_GPIO_PIN_3   /* PD3  */
#define HW_UART_GPS_GPIO_AF                LL_GPIO_AF_7

/* UART2: ELRS 2.4G Receiver */
#define HW_UART_ELRS_INSTANCE              USART2
#define HW_UART_ELRS_GPIO_PORT             GPIOA
#define HW_UART2_BAUDRATE                  420000U
#define HW_UART_ELRS_RX_PIN                LL_GPIO_PIN_3   /* PA3  */
#define HW_UART_ELRS_TX_PIN                LL_GPIO_PIN_2   /* PA2  */
#define HW_UART_ELRS_GPIO_AF               LL_GPIO_AF_7

/* UART1: Reserved */
#define HW_UART_DEBUG_INSTANCE             USART1
#define HW_UART_DEBUG_GPIO_PORT            GPIOA
#define HW_UART_DEBUG_BAUDRATE             921600U
#define HW_UART_DEBUG_TX_PIN               LL_GPIO_PIN_9   /* PA9  */
#define HW_UART_DEBUG_RX_PIN               LL_GPIO_PIN_10  /* PA10 */
#define HW_UART_DEBUG_GPIO_AF              LL_GPIO_AF_7

/* Timers: ESC DSHOT */
#define HW_DSHOT_MOTOR_NUM                 4U
#define HW_TIM_DSHOT_INSTANCE              TIM1
#define HW_TIM_DSHOT_GPIO_PORT             GPIOE
#define HW_TIM_DSHOT_CH1_PIN               LL_GPIO_PIN_9   /* PE9  */
#define HW_TIM_DSHOT_CH2_PIN               LL_GPIO_PIN_11  /* PE11 */
#define HW_TIM_DSHOT_CH3_PIN               LL_GPIO_PIN_13  /* PE13 */
#define HW_TIM_DSHOT_CH4_PIN               LL_GPIO_PIN_14  /* PE14 */
#define HW_TIM_DSHOT_GPIO_AF               LL_GPIO_AF_1
#define HW_TIM_DSHOT_ARR                   ((280U * 2U) - 1U)
#define HW_TIM_DSHOT_BIT_0                 (104U * 2U)
#define HW_TIM_DSHOT_BIT_1                 (210U * 2U)

#define HW_TIM_STAMP_INSTANCE              TIM5
#define HW_TIM_STAMP_PSC                   (84U - 1U)

/* DMA mapping */
#define HW_DMA_DSHOT_INSTANCE              DMA2
#define HW_DMA_DSHOT_STREAM                LL_DMA_STREAM_5
#define HW_DMA_DSHOT_CHANNEL               LL_DMA_CHANNEL_6

#define HW_DMA_SPI1_RX_INSTANCE            DMA2
#define HW_DMA_SPI1_RX_STREAM              LL_DMA_STREAM_0
#define HW_DMA_SPI1_RX_CHANNEL             LL_DMA_CHANNEL_3
#define HW_DMA_SPI1_TX_INSTANCE            DMA2
#define HW_DMA_SPI1_TX_STREAM              LL_DMA_STREAM_3
#define HW_DMA_SPI1_TX_CHANNEL             LL_DMA_CHANNEL_3

#define HW_DMA_SPI2_RX_INSTANCE            DMA1
#define HW_DMA_SPI2_RX_STREAM              LL_DMA_STREAM_3
#define HW_DMA_SPI2_RX_CHANNEL             LL_DMA_CHANNEL_0
#define HW_DMA_SPI2_TX_INSTANCE            DMA1
#define HW_DMA_SPI2_TX_STREAM              LL_DMA_STREAM_4
#define HW_DMA_SPI2_TX_CHANNEL             LL_DMA_CHANNEL_0

#define HW_DMA_IIC2_RX_INSTANCE            DMA1
#define HW_DMA_IIC2_RX_STREAM              LL_DMA_STREAM_2
#define HW_DMA_IIC2_RX_CHANNEL             LL_DMA_CHANNEL_7

#define HW_DMA_UART2_RX_INSTANCE           DMA1
#define HW_DMA_UART2_RX_STREAM             LL_DMA_STREAM_5
#define HW_DMA_UART2_RX_CHANNEL            LL_DMA_CHANNEL_4

#define HW_DMA_UART2_TX_INSTANCE           DMA1
#define HW_DMA_UART2_TX_STREAM             LL_DMA_STREAM_6
#define HW_DMA_UART2_TX_CHANNEL            LL_DMA_CHANNEL_4

#define HW_DMA_UART3_RX_INSTANCE           DMA1 
#define HW_DMA_UART3_RX_STREAM             LL_DMA_STREAM_1
#define HW_DMA_UART3_RX_CHANNEL            LL_DMA_CHANNEL_4

#define HW_DMA_TIM1_INSTANCE               DMA2
#define HW_DMA_TIM1_STREAM                 LL_DMA_STREAM_5
#define HW_DMA_TIM1_CHANNEL                LL_DMA_CHANNEL_6

#define HW_DMA_SDIO_INSTANCE               DMA2
#define HW_DMA_SDIO_STREAM                 LL_DMA_STREAM_6     
#define HW_DMA_SDIO_CHANNEL                LL_DMA_CHANNEL_4

/*Device ID*/
#define DEVICE_SPI1                        0U
#define DEVICE_SPI2                        1U
#define DEVICE_SPI3                        2U

#define DEVICE_IIC1                        0U
#define DEVICE_IIC2                        1U
#define DEVICE_IIC3                        2U

/*EXTI_Line ID*/
#define EXTI_LINE_0_ID                     0U
#define EXTI_LINE_1_ID                     1U
#define EXTI_LINE_2_ID                     2U
#define EXTI_LINE_3_ID                     3U
#define EXTI_LINE_4_ID                     4U
#define EXTI_LINE_5_ID                     5U

/*Device EXTI_Line ID*/
#define BMM150_EXTI_LINE                   EXTI_LINE_1_ID  
#define ICM42688_EXTI_LINE                 EXTI_LINE_0_ID
#define BMP390L_EXTI_LINE                  EXTI_LINE_5_ID

/* IRQ mapping */
#define HW_IRQ_EXTI_ICM42688               EXTI0_IRQn
#define HW_IRQ_EXTI_BMM150                 EXTI1_IRQn
#define HW_IRQ_EXTI_BMP390L                EXTI9_5_IRQn
#define HW_IRQ_DMA_DSHOT                   DMA2_Stream5_IRQn
#define HW_IRQ_DMA_SPI1_RX                 DMA2_Stream0_IRQn
#define HW_IRQ_DMA_SPI2_RX                 DMA1_Stream3_IRQn
#define HW_IRQ_DMA_IIC2_RX                 DMA1_Stream2_IRQn
#define HW_IRQ_UART2_IDLE                  USART2_IRQn
#define HW_IRQ_DMA_UART2_TX                DMA1_Stream6_IRQn
#define HW_IRQ_DMA_SDIO                    DMA2_Stream6_IRQn
#define HW_IRQ_AHRS_TASK                   EXTI2_IRQn


/* IRQ Priorities */
#define HW_DMA_TO_SPI1_IRQ_PRIORITY        2U
#define HW_DMA_TO_SPI2_IRQ_PRIORITY        1U
#define HW_DMA_TO_IIC2_IRQ_PRIORITY        2U   
#define HW_EXTI_BMM150_IRQ_PRIORITY        2U
#define HW_EXTI_ICM42688_IRQ_PRIORITY      1U
#define HW_EXTI_BMP390L_IRQ_PRIORITY       3U
#define HW_UART2_IRQ_PRIORITY              4U  
#define HW_DMA_UART2_TX_IRQ_PRIORITY       6U  
#define HW_DMA_DSHOT_IRQ_PRIORITY          1U
#define HW_DMA_SDIO_IRQ_PRIORITY           8U
#define HW_AHRS_TASK_PRIORITY              2U

/* Core clock target */
#define HW_SYS_CORE_CLOCK_HZ               168000000UL
#define HW_SYS_HSE_CLOCK_HZ                8000000UL

#endif /* __SYS_HARDWARE_CONFIG_H */