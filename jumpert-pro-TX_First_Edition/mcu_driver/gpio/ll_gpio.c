#include "gpio/ll_gpio.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

void UART_GPIO_Init(void)
{
    //UART1_GPIO_Init
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef UART1_GPIO_InitStructure = {0}; 

    UART1_GPIO_InitStructure.Pin = HW_UART_RF_TX_PIN | HW_UART_RF_RX_PIN;
    UART1_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    UART1_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    UART1_GPIO_InitStructure.Alternate = HW_UART_RF_GPIO_AF;
    UART1_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    UART1_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_UART_RF_GPIO_PORT , &UART1_GPIO_InitStructure); 

    // Debug UART GPIO Init (USART2 on PD5/PD6)
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

    LL_GPIO_InitTypeDef UART_DBG_GPIO_InitStructure = {0}; 

    UART_DBG_GPIO_InitStructure.Pin = HW_UART_DBG_RX_PIN | HW_UART_DBG_TX_PIN;
    UART_DBG_GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    UART_DBG_GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    UART_DBG_GPIO_InitStructure.Alternate = HW_UART_DBG_GPIO_AF;
    UART_DBG_GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    UART_DBG_GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(HW_UART_DBG_GPIO_PORT , &UART_DBG_GPIO_InitStructure); 

}


void OLED_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.Pin = HW_I2C_OLED_SCL_PIN | HW_I2C_OLED_SDA_PIN;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStructure.Alternate = HW_I2C_OLED_GPIO_AF; 
    LL_GPIO_Init(HW_I2C_OLED_GPIO_PORT, &GPIO_InitStructure);
}


void five_way_key_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); 

    LL_GPIO_InitTypeDef key_gpio_initstructure = {0}; 

    key_gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
    key_gpio_initstructure.Pin = HW_KEY_UP_PIN;
    key_gpio_initstructure.Pull = LL_GPIO_PULL_UP;
    key_gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_LOW;

    LL_GPIO_Init(HW_KEY_UP_PORT , &key_gpio_initstructure); 

    key_gpio_initstructure.Pin = HW_KEY_DOWN_PIN; 
    LL_GPIO_Init(HW_KEY_DOWN_PORT , &key_gpio_initstructure); 

    key_gpio_initstructure.Pin = HW_KEY_ENTER_PIN; 
    LL_GPIO_Init(HW_KEY_ENTER_PORT , &key_gpio_initstructure); 

    key_gpio_initstructure.Pin = HW_KEY_BACK_PIN; 
    LL_GPIO_Init(HW_KEY_BACK_PORT , &key_gpio_initstructure); 

    key_gpio_initstructure.Pin = HW_KEY_RETURN_PIN; 
    LL_GPIO_Init(HW_KEY_RETURN_PORT , &key_gpio_initstructure); 
}

void JoyStick_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef joystick_gpio_initstructure = {0} ; 

    joystick_gpio_initstructure.Mode = LL_GPIO_MODE_ANALOG;
    joystick_gpio_initstructure.Pin = HW_ADC_JOY_PITCH_PIN;
    joystick_gpio_initstructure.Pull = LL_GPIO_PULL_NO;
    joystick_gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; 

    LL_GPIO_Init(HW_ADC_JOY_PITCH_PORT , &joystick_gpio_initstructure) ; 

    joystick_gpio_initstructure.Pin = HW_ADC_JOY_ROLL_PIN ; 
    LL_GPIO_Init(HW_ADC_JOY_ROLL_PORT , &joystick_gpio_initstructure) ; 

    joystick_gpio_initstructure.Pin = HW_ADC_JOY_YAW_PIN ; 
    LL_GPIO_Init(HW_ADC_JOY_YAW_PORT , &joystick_gpio_initstructure) ; 

    joystick_gpio_initstructure.Pin = HW_ADC_JOY_THROTTLE_PIN ; 
    LL_GPIO_Init(HW_ADC_JOY_THROTTLE_PORT , &joystick_gpio_initstructure) ; 
}

void TWO_Stage_Lever_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA) ;
    
    LL_GPIO_InitTypeDef two_stage_lever_gpio_initstructure = {0} ; 

    two_stage_lever_gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
    two_stage_lever_gpio_initstructure.Pin = HW_TWO_LEVER_GPIO_PIN;
    two_stage_lever_gpio_initstructure.Pull = LL_GPIO_PULL_NO;
    two_stage_lever_gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;

    LL_GPIO_Init(HW_TWO_LEVER_GPIO_PORT , &two_stage_lever_gpio_initstructure) ; 
}

void THREE_Stage_Lever_GPIO_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA) ;
    
    LL_GPIO_InitTypeDef three_stage_lever_gpio_initstructure = {0} ; 

    three_stage_lever_gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
    three_stage_lever_gpio_initstructure.Pin = HW_THREE_LEVER_GPIO_PIN_1 | HW_THREE_LEVER_GPIO_PIN_2;
    three_stage_lever_gpio_initstructure.Pull = LL_GPIO_PULL_UP;
    three_stage_lever_gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;

    LL_GPIO_Init(HW_THREE_LEVER_GPIO_PORT , &three_stage_lever_gpio_initstructure) ; 
}