#include "pid_trace/pid_trace_port_usart1.h"

#if PIDTRACE_ENABLE

#include "pid_trace/pid_trace_port.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_usart.h"

static uint16_t usart1_blocking_write(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0U;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    while (sent < len) {
        while (!LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE)) {
        }
        LL_USART_TransmitData8(HW_UART_DEBUG_INSTANCE, data[sent]);
        sent++;
    }

    while (!LL_USART_IsActiveFlag_TC(HW_UART_DEBUG_INSTANCE)) {
    }

    return sent;
}

static uint8_t usart1_is_busy(void)
{
    return LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE) ? 0U : 1U;
}

static const PidTrace_ExportSink_t s_usart1_export = {
    .write = usart1_blocking_write,
    .is_busy = usart1_is_busy,
};

void PidTrace_PortUSART1_Register(void)
{
    PidTrace_RegisterExportSink(&s_usart1_export);
}

#endif /* PIDTRACE_ENABLE */
