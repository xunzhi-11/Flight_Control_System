#include "gpio/ll_gpio_bmm150.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

void gpio_spi1_Init(void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA) ;
    
    LL_GPIO_InitTypeDef SPI1_GPIO_InitStructure = {0} ;
    
    SPI1_GPIO_InitStructure.Alternate = HW_SPI_BMM150_GPIO_AF ;
    SPI1_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE ;
    SPI1_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL ;
    SPI1_GPIO_InitStructure.Pin = HW_SPI_BMM150_SCK_PIN | HW_SPI_BMM150_MOSI_PIN ;
    SPI1_GPIO_InitStructure.Pull = LL_GPIO_PULL_NO ;
    SPI1_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH ;
    
    LL_GPIO_Init(HW_SPI_BMM150_GPIO_PORT , &SPI1_GPIO_InitStructure) ;

    SPI1_GPIO_InitStructure.Pin = HW_SPI_BMM150_MISO_PIN  ;
    SPI1_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP ;
    LL_GPIO_Init(HW_SPI_BMM150_GPIO_PORT , &SPI1_GPIO_InitStructure) ;
}

void gpio_bmm150_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA) ;
    
    LL_GPIO_InitTypeDef GPIO_InitStructure = {0} ;
    
    GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT ;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL ;
    GPIO_InitStructure.Pin = HW_GPIO_BMM150_PS_PIN ; 
    GPIO_InitStructure.Pull = LL_GPIO_PULL_NO ;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH ;
    
    LL_GPIO_Init(HW_GPIO_BMM150_PS_PORT , &GPIO_InitStructure) ;
    LL_GPIO_ResetOutputPin (HW_GPIO_BMM150_PS_PORT , HW_GPIO_BMM150_PS_PIN) ;

       
    GPIO_InitStructure.Pin = HW_GPIO_BMM150_CS_PIN ; 
    GPIO_InitStructure.Pull = LL_GPIO_PULL_UP ;
    LL_GPIO_Init(HW_GPIO_BMM150_CS_PORT , &GPIO_InitStructure) ;
    LL_GPIO_SetOutputPin (HW_GPIO_BMM150_CS_PORT , HW_GPIO_BMM150_CS_PIN) ;
}

void gpio_bmm150_drdy_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA) ; 
    
    LL_GPIO_InitTypeDef bmm150_gpio_drdy_initstructure = {0} ; 
    
    bmm150_gpio_drdy_initstructure.Mode = LL_GPIO_MODE_INPUT ;
    bmm150_gpio_drdy_initstructure.Pin = HW_GPIO_BMM150_DRDY_PIN ;
    bmm150_gpio_drdy_initstructure.Pull = LL_GPIO_PULL_NO ;
    bmm150_gpio_drdy_initstructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM ; 

    LL_GPIO_Init( HW_GPIO_BMM150_DRDY_PORT, &bmm150_gpio_drdy_initstructure) ; 
}

