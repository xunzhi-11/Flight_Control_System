#include "gpio/ll_gpio_icm42688.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

void gpio_spi2_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB) ;
  
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0} ;
    
    GPIO_InitStruct.Pin = HW_SPI_ICM42688_SCK_PIN | HW_SPI_ICM42688_MOSI_PIN ;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE ;
    GPIO_InitStruct.Alternate = HW_SPI_ICM42688_GPIO_AF ;  
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH ;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL ;  
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO ;
    LL_GPIO_Init(HW_SPI_ICM42688_GPIO_PORT, &GPIO_InitStruct) ;
   
    GPIO_InitStruct.Pin = HW_SPI_ICM42688_MISO_PIN ;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;  
    LL_GPIO_Init(HW_SPI_ICM42688_GPIO_PORT, &GPIO_InitStruct) ;
}

void gpio_icm42688_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB) ;
 
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0} ;
    
    GPIO_InitStruct.Pin = HW_GPIO_ICM42688_CS_PIN ;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT ;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH ;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL ;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
    
    LL_GPIO_Init(HW_GPIO_ICM42688_CS_PORT, &GPIO_InitStruct) ;

    LL_GPIO_SetOutputPin(HW_GPIO_ICM42688_CS_PORT, HW_GPIO_ICM42688_CS_PIN) ;
}

void gpio_icm42688_drdy_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC) ; 
    
    LL_GPIO_InitTypeDef icm42688_gpio_drdy_initstructure = {0} ; 
    
    icm42688_gpio_drdy_initstructure.Mode = LL_GPIO_MODE_INPUT ;
    icm42688_gpio_drdy_initstructure.Pin = HW_GPIO_ICM42688_DRDY_PIN ;
    icm42688_gpio_drdy_initstructure.Pull = LL_GPIO_PULL_NO ;
    icm42688_gpio_drdy_initstructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM ; 

    LL_GPIO_Init( HW_GPIO_ICM42688_DRDY_PORT, &icm42688_gpio_drdy_initstructure) ; 
}