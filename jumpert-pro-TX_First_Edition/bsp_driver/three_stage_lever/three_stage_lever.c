#include "three_stage_lever/three_stage_lever.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_gpio.h"

three_lever_mode_e get_three_lever_mode(void)
{
    uint8_t pa6 = LL_GPIO_IsInputPinSet(HW_THREE_LEVER_GPIO_PORT , HW_THREE_LEVER_GPIO_PIN_1) ? 1U : 0U ;
    uint8_t pa7 = LL_GPIO_IsInputPinSet(HW_THREE_LEVER_GPIO_PORT , HW_THREE_LEVER_GPIO_PIN_2) ? 1U : 0U ;
    uint8_t level = (uint8_t)((pa7 << 1U) | pa6) ;

    switch (level)
    {
        case 0b01 :
        return THREE_LEVER_MODE_HOVER ;

        case 0b10 :
        return THREE_LEVER_MODE_STATIC ;

        case 0b11 :
        return THREE_LEVER_MODE_MANUAL ;

        default :
        return THREE_LEVER_MODE_MANUAL ;
    }
}
