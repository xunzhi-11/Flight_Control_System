#include "two_stage_lever/two_statge_lever.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_gpio.h"
#include "stdbool.h"


bool is_ready_to_unlock(void)
{
    switch(LL_GPIO_IsInputPinSet(HW_TWO_LEVER_GPIO_PORT , HW_TWO_LEVER_GPIO_PIN))
    {
        case 0 :
        return true ; 

        case 1 :
        return false ; 

        default :
        return false ; 
    }
}
