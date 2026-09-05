#include "iic/ll_i2c.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_i2c.h"
#include "sys_hardware_config.h"

void iic_bmp390l_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2) ; 

    LL_I2C_InitTypeDef iic_bmp390l_initstructure = {0} ; 

    iic_bmp390l_initstructure.ClockSpeed = HW_I2C_BMP390L_SPEED;
    iic_bmp390l_initstructure.DutyCycle = LL_I2C_DUTYCYCLE_2;
    iic_bmp390l_initstructure.OwnAddress1 = 0x00;
    iic_bmp390l_initstructure.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    iic_bmp390l_initstructure.PeripheralMode = LL_I2C_MODE_I2C;
    iic_bmp390l_initstructure.TypeAcknowledge = LL_I2C_ACK;

    LL_I2C_Init(HW_I2C_BMP390L_INSTANCE , &iic_bmp390l_initstructure) ; 
    LL_I2C_Enable(HW_I2C_BMP390L_INSTANCE) ;
}