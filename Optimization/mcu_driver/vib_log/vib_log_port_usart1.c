#include "vib_log/vib_log_port_usart1.h"

#if VIBLOG_ENABLE

#include "vib_log/vib_log_port.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_usart.h"

#ifndef VIBLOG_USART1_RING_SIZE
#define VIBLOG_USART1_RING_SIZE (4096*5)
#endif

static uint8_t s_ring[VIBLOG_USART1_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static uint16_t ring_free_space(void)
{
    const uint16_t head = s_head;
    const uint16_t tail = s_tail;

    if (head >= tail) {
        return (uint16_t)(VIBLOG_USART1_RING_SIZE - (head - tail) - 1U);
    }

    return (uint16_t)(tail - head - 1U);
}

static void usart1_poll(void)
{
    while (s_head != s_tail && LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE)) {
        LL_USART_TransmitData8(HW_UART_DEBUG_INSTANCE, s_ring[s_tail]);
        s_tail = (uint16_t)((s_tail + 1U) % VIBLOG_USART1_RING_SIZE);
    }
}

static uint16_t usart1_write(const uint8_t *data, uint16_t len)
{
    uint16_t accepted = 0U;
    const uint16_t free_space = ring_free_space();

    if (data == NULL || len == 0U) {
        return 0U;
    }

    /* All-or-nothing: partial frame writes produce VB-only garbage on the wire. */
    if (free_space < len) {
        usart1_poll();
        if (ring_free_space() < len) {
            return 0U;
        }
    }

    while (accepted < len) {
        s_ring[s_head] = data[accepted];
        s_head = (uint16_t)((s_head + 1U) % VIBLOG_USART1_RING_SIZE);
        accepted++;
    }

    usart1_poll();
    return accepted;
}

static const VibLog_Sink_t s_usart1_sink = {
    .write = usart1_write,
    .poll = usart1_poll,
};

void VibLog_PortUSART1_Register(void)
{
    s_head = 0U;
    s_tail = 0U;
    VibLog_RegisterSink(&s_usart1_sink);
}

#endif /* VIBLOG_ENABLE */
