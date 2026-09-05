/*
 * BMP390L DRDY 软件模拟（临时）
 *
 * 背景：BMP390L INT 引脚损坏，无法输出 DRDY。
 * 做法：PC1 输出 20Hz 方波，硬件短接 PC1 -> PB5，复用现有 PB5 上升沿 EXTI 链路。
 *
 * 删除步骤：
 *   1. 删除本文件与 bmp390l_drdy_sim.h
 *   2. 去掉 System_Application.c 中的 #include 与两处调用
 *   3. 去掉 .eide/eide.yml 中 User/bmp390l_drdy_sim.c 条目
 *   4. 断开 PC1-PB5 短接线
 */

#include "bmp390l_drdy_sim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

#define BMP390L_DRDY_SIM_GPIO_PORT      GPIOC
#define BMP390L_DRDY_SIM_GPIO_PIN       LL_GPIO_PIN_1

/* 20Hz 方波：半周期 25ms */
#define BMP390L_DRDY_SIM_HALF_PERIOD_MS 25U
#define BMP390L_DRDY_SIM_TASK_STACK     256U
#define BMP390L_DRDY_SIM_TASK_PRIO      6U

static TaskHandle_t s_drdy_sim_task = NULL;

static void bmp390l_drdy_sim_gpio_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BMP390L_DRDY_SIM_GPIO_PIN;
    gpio.Mode = LL_GPIO_MODE_OUTPUT;
    gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BMP390L_DRDY_SIM_GPIO_PORT, &gpio);

    LL_GPIO_ResetOutputPin(BMP390L_DRDY_SIM_GPIO_PORT, BMP390L_DRDY_SIM_GPIO_PIN);
}

static void bmp390l_drdy_sim_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t half_period = pdMS_TO_TICKS(BMP390L_DRDY_SIM_HALF_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&last_wake, half_period);
        LL_GPIO_TogglePin(BMP390L_DRDY_SIM_GPIO_PORT, BMP390L_DRDY_SIM_GPIO_PIN);
    }
}

void bmp390l_drdy_sim_hw_init(void)
{
    bmp390l_drdy_sim_gpio_init();
}

void bmp390l_drdy_sim_task_start(void)
{
    if (s_drdy_sim_task != NULL) {
        return;
    }

    xTaskCreate(
        bmp390l_drdy_sim_task,
        "bmp390_drdy_sim",
        BMP390L_DRDY_SIM_TASK_STACK,
        NULL,
        BMP390L_DRDY_SIM_TASK_PRIO,
        &s_drdy_sim_task);
}
