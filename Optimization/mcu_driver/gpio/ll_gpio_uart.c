#include "gpio/ll_gpio_uart.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

void gpio_uart_init(void)
{
    //UART1_GPIO_Init
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef UART1_GPIO_InitStructure = {0} ; 

    UART1_GPIO_InitStructure.Pin = HW_UART_DEBUG_TX_PIN | HW_UART_DEBUG_RX_PIN;
    UART1_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    UART1_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    UART1_GPIO_InitStructure.Alternate = HW_UART_DEBUG_GPIO_AF;
    UART1_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    UART1_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_UART_DEBUG_GPIO_PORT , &UART1_GPIO_InitStructure) ; 

    //UART2_GPIO_Init
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef UART2_GPIO_InitStructure = {0} ; 

    UART2_GPIO_InitStructure.Pin = HW_UART_ELRS_TX_PIN | HW_UART_ELRS_RX_PIN;
    UART2_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    UART2_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    UART2_GPIO_InitStructure.Alternate = HW_UART_ELRS_GPIO_AF;
    UART2_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    UART2_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_UART_ELRS_GPIO_PORT , &UART2_GPIO_InitStructure) ; 

    //UART3_GPIO_Init
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

    LL_GPIO_InitTypeDef UART3_GPIO_InitStructure = {0} ; 

    UART3_GPIO_InitStructure.Pin = HW_UART_GPS_TX_PIN | HW_UART_GPS_RX_PIN;
    UART3_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    UART3_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    UART3_GPIO_InitStructure.Alternate = HW_UART_GPS_GPIO_AF;
    UART3_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    UART3_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_UART_GPS_GPIO_PORT , &UART3_GPIO_InitStructure) ; 
    

}
