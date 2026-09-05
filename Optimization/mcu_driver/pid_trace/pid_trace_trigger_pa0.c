#include "pid_trace/pid_trace_trigger_pa0.h"

#if PIDTRACE_ENABLE

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

void PidTrace_PortTrigger_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
}

uint8_t PidTrace_PortTrigger_IsPressed(void)
{
    return LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 1U : 0U;
}

#endif /* PIDTRACE_ENABLE */
