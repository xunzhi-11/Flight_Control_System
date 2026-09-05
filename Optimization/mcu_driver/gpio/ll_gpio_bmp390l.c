#include "gpio/ll_gpio_bmp390l.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

void gpio_i2c2_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB) ; 

    LL_GPIO_InitTypeDef gpio_i2c2_initstructure = {0} ; 

    gpio_i2c2_initstructure.Alternate = HW_I2C_BMP390L_GPIO_AF;
    gpio_i2c2_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_i2c2_initstructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_i2c2_initstructure.Pin = HW_I2C_BMP390L_SCL_PIN | HW_I2C_BMP390L_SDA_PIN;
    gpio_i2c2_initstructure.Pull = LL_GPIO_PULL_UP;
    gpio_i2c2_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_I2C_BMP390L_GPIO_PORT , &gpio_i2c2_initstructure) ; 
}

void gpio_bmp390l_drdy_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB) ; 

    LL_GPIO_InitTypeDef bmp390l_gpio_drdy_initstructure = {0} ; 

    bmp390l_gpio_drdy_initstructure.Mode = LL_GPIO_MODE_INPUT ;
    bmp390l_gpio_drdy_initstructure.Pin = HW_GPIO_BMP390L_DRDY_PIN ;
    bmp390l_gpio_drdy_initstructure.Pull = LL_GPIO_PULL_DOWN ;
    bmp390l_gpio_drdy_initstructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM ; 

    LL_GPIO_Init( HW_GPIO_BMP390L_DRDY_PORT, &bmp390l_gpio_drdy_initstructure) ; 
}