#include "5_way_key/5_way_key.h"
#include "system_hardware_config.h"
#include "stm32f4xx_ll_gpio.h"

#define LONG_JUDG_THRESHOLD  100

// 硬件读取函数指针原型
typedef uint8_t (*HW_Read_Func_t)(void) ;

typedef enum
{
    SPARE = 0 , 
    DEBOUNCE = 1 , 
    COUNTING = 2 , 
    WAIT_RELEASE = 3
} Key_State_e ; 

// 把硬件动作和语义绑定起来的结构体
typedef struct {
    HW_Read_Func_t read_hw_level ;     // 接口：提供如何读取这个按键的物理电平
    Input_Action_e   action_short ;      // 映射：短按时产生语义
    Input_Action_e   action_long ;       // 映射：长按时产生语义
    
    // 引擎私有的状态机变量
    Key_State_e        state ;             // 防抖状态
    uint16_t       tick_count ;        // 计时器
} KeyNode_t ;

static uint8_t Read_Pin_UP(void)    { return LL_GPIO_IsInputPinSet(HW_KEY_UP_PORT, HW_KEY_UP_PIN) ; }
static uint8_t Read_Pin_DOWN(void)  { return LL_GPIO_IsInputPinSet(HW_KEY_DOWN_PORT, HW_KEY_DOWN_PIN) ; }
static uint8_t Read_Pin_ENTER(void) { return LL_GPIO_IsInputPinSet(HW_KEY_ENTER_PORT, HW_KEY_ENTER_PIN) ; }
static uint8_t Read_Pin_BACK(void)  { return LL_GPIO_IsInputPinSet(HW_KEY_BACK_PORT, HW_KEY_BACK_PIN) ; }
static uint8_t Read_Pin_RETURN(void)  { return LL_GPIO_IsInputPinSet(HW_KEY_RETURN_PORT, HW_KEY_RETURN_PIN) ; }

KeyNode_t g_KeyList[] = {
    // 读哪个函数       短按发什么语义      长按发什么语义 (不需要长按就填 NONE)
    {Read_Pin_UP,      ACTION_UP,        ACTION_NONE,    SPARE, 0},
    {Read_Pin_DOWN,    ACTION_DOWN,      ACTION_NONE,    SPARE, 0},
    {Read_Pin_ENTER,   ACTION_ENTER,     ACTION_NONE,    SPARE, 0},
    {Read_Pin_BACK,    ACTION_BACK,      ACTION_NONE,    SPARE, 0},
    {Read_Pin_RETURN,    ACTION_RETURN,      ACTION_NONE,    SPARE, 0}
} ;

// 引擎按键数量
const uint8_t g_KeyCount = sizeof(g_KeyList) / sizeof(KeyNode_t) ;
static Key_Event_Cb_t g_key_event_cb = NULL ;

void Key_RegisterCallback(Key_Event_Cb_t cb)
{
    g_key_event_cb = cb ;
}


static uint8_t Read_Pin(void)
{
    uint8_t Pin_value = 0 ; 
    for(int i = 0 ; i < g_KeyCount ; i++)
    {
        Pin_value |= (g_KeyList[i].read_hw_level() << i) ; 
    }
    return Pin_value ; 
}


void Key_Scan(void)
{
    uint8_t Pin_value = Read_Pin() ; 

    for(int i = 0 ; i < g_KeyCount ; i++)
    {
        uint8_t current_level = (Pin_value >> i) & 0x01 ;
        switch (g_KeyList[i].state)
        {
        case SPARE:
            if(current_level == 0)
            {
                g_KeyList[i].state = DEBOUNCE ; 
            }
        break ;

        case DEBOUNCE:
            if(current_level == 0)
            {
                g_KeyList[i].state = COUNTING ; 
                g_KeyList[i].tick_count = 0 ;
            }
            else
            {
                g_KeyList[i].state = SPARE ; 
            }
        break ;

        case COUNTING: 
            if(current_level == 0) 
            {
                g_KeyList[i].tick_count++ ;
                if(g_KeyList[i].tick_count == LONG_JUDG_THRESHOLD)
                {
                    if (g_key_event_cb != NULL && g_KeyList[i].action_long != ACTION_NONE) {
                        g_key_event_cb(g_KeyList[i].action_long) ;
                    }
                    g_KeyList[i].state = WAIT_RELEASE ; 
                }
            }
            else 
            {
                if (g_key_event_cb != NULL && g_KeyList[i].action_short != ACTION_NONE) {
                    g_key_event_cb(g_KeyList[i].action_short) ;
                }
                g_KeyList[i].state = SPARE ;
            }
        break ;
        
        case WAIT_RELEASE:
            if(current_level == 1) 
            {
                g_KeyList[i].state = SPARE ; 
            }
        break ; 

        default:
        break ;
        }
    }
}