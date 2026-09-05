#include "gpio/ll_gpio_dshot.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "sys_hardware_config.h"

void gpio_dshot_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE); 
    
    LL_GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL; 
    GPIO_InitStructure.Alternate = HW_TIM_DSHOT_GPIO_AF; // TIM1 AF
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    
    uint32_t motor_pins[4] = {HW_TIM_DSHOT_CH1_PIN, HW_TIM_DSHOT_CH2_PIN, HW_TIM_DSHOT_CH3_PIN, HW_TIM_DSHOT_CH4_PIN};
    for(int i=0; i<4; i++) 
    {
        GPIO_InitStructure.Pin = motor_pins[i];
        LL_GPIO_Init(HW_TIM_DSHOT_GPIO_PORT, &GPIO_InitStructure);
    }

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
}