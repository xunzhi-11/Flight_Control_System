#include "app_task/app_task_UI.h"
#include "app_task/app_task_input_scan.h"
#include "app_core/ui_menu_layout.h" 
#include "app_core/ui_status_page.h"
#include "app_core/joy_calib_flow.h"
#include "OLED/OLED.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t UI_taskHandle = NULL ; 

static MenuPage_t *current_page = &page_status ; 

static uint8_t cursor_index = 0 ;      // 光标指向当前页面的第几个条目
static uint8_t window_top_index = 0 ;  // 屏幕最顶端显示的条目索引
#define MAX_VISIBLE_ITEMS 3           // OLED除去标题栏，最多显示 3 行菜单

static char* s_toast_msg = NULL ;
static uint16_t s_toast_timer_ms = 0 ;
#define TIME_TO_SHOW_TIP 1000U

void UI_Show_Toast(char* msg) 
{
    s_toast_msg = msg ;
    s_toast_timer_ms = TIME_TO_SHOW_TIP ;
}

static void UI_Enter_Submenu(MenuPage_t *submenu)
{
    if (submenu != NULL)
    {
        current_page = submenu ;
        cursor_index = 0 ;
        window_top_index = 0 ;
    }
}

static void UI_Process_Action(Input_Action_e action)
{
    if (current_page->page_type == PAGE_TYPE_STATUS)
    {
        switch (action)
        {
            case ACTION_ENTER:
                if (current_page->item_count > 0 &&
                    current_page->items[0].type == ITEM_TYPE_SUBMENU)
                {
                    UI_Enter_Submenu(current_page->items[0].payload.submenu) ;
                }
                break ;

            default:
                break ;
        }
        return ;
    }

    switch (action)
    {
        case ACTION_UP:
            if (cursor_index > 0) {
                cursor_index-- ;
                // 向上推滑动窗口
                if (cursor_index < window_top_index) {
                    window_top_index = cursor_index ;
                }
            }
            break ;

        case ACTION_DOWN:
            if (cursor_index < current_page->item_count - 1) {
                cursor_index++ ;
                // 向下拉滑动窗口
                if (cursor_index >= window_top_index + MAX_VISIBLE_ITEMS) {
                    window_top_index = cursor_index - MAX_VISIBLE_ITEMS + 1 ;
                }
            }
            break ;

        case ACTION_BACK:
            if (current_page->parent != NULL) {
                current_page = current_page->parent ; // 返回上一级
                cursor_index = 0 ;      // 状态复位
                window_top_index = 0 ;  
            }
            break ;

        case ACTION_ENTER:
            {
                // 获取当前光标选中的条目
                MenuItem_t *selected = &current_page->items[cursor_index] ;
                
                if (selected->type == ITEM_TYPE_SUBMENU) {
                    UI_Enter_Submenu(selected->payload.submenu) ;
                } 
                else if (selected->type == ITEM_TYPE_ACTION) {
                    // 访问内部callback
                    if (selected->payload.action.callback != NULL) {
                        selected->payload.action.callback(selected->payload.action.arg) ;
                    }
                }
            }
            break ;

        case ACTION_RETURN :
            if (current_page != &page_status) {
                current_page = &page_status ;
                cursor_index = 0 ;
                window_top_index = 0 ;
            }
        break ; 

        default:
            break ;
    }
}

static void UI_Render_Menu_Page(void)
{
    OLED_Clear() ; // 极速清空显存

    // 画标题栏 (第 1 行)
    OLED_ShowString(1, 1, (char *)current_page->title) ;

    if (window_top_index > 0) 
    {
        OLED_ShowString(2, 15, "^") ; 
    }
    // 如果下面还有条目没显示出来，在右下角画个向下的箭头
    if (window_top_index + MAX_VISIBLE_ITEMS < current_page->item_count) 
    {
        OLED_ShowString(4, 15, "v") ;
    }

    // 遍历当前滑动窗口内的条目并渲染 (第 2~4 行)
    for (uint8_t i = 0 ; i < MAX_VISIBLE_ITEMS ; i++) 
    {
        uint8_t item_idx = window_top_index + i ;
        
        // 如果列表没这么长，直接跳出循环
        if (item_idx >= current_page->item_count) break ; 
        
        MenuItem_t *item = &current_page->items[item_idx] ;
        uint8_t display_line = i + 2 ; // 从第 2 行开始显示

        // 如果是当前选中的行，画个光标 '>'
        if (item_idx == cursor_index) {
            OLED_ShowString(display_line, 1, ">") ; 
            OLED_ShowString(display_line, 2, (char *)item->name) ;
        } else {
            OLED_ShowString(display_line, 2, (char *)item->name) ;
        }
    }

    if (s_toast_timer_ms > 0 && s_toast_msg != NULL) 
    {
        OLED_ShowString(3, 1, "                ") ; // 清空该行
        OLED_ShowString(3, 1, s_toast_msg) ;        // 显示提示信息
    }

    OLED_Update() ; // 触发 DMA 异步推屏
}

static void UI_Render_Current_Page(void)
{
    if (current_page->page_type == PAGE_TYPE_STATUS)
    {
        UI_Status_Render() ;
        return ;
    }

    UI_Render_Menu_Page() ;
}

static void Task_UI(void* pvParameters)
{
    // 初始化菜单树的父子关系
    UI_Menu_Init() ; 

    Input_Action_e action ;
    
    while(1)
    {
        if (JoyCalib_Is_Active())
        {
            if (Input_Get_Action(&action, 50))
            {
                JoyCalib_Process(action) ;
            }

            JoyCalib_Tick() ;
            JoyCalib_Render() ;
            continue ;
        }

        // 阻塞等待 50ms (20Hz 刷新率)
        if (Input_Get_Action(&action, 50)) 
        {
            UI_Process_Action(action) ; 
        }
        
        if (s_toast_timer_ms > 0) 
        {
            if (s_toast_timer_ms > 50) 
            {
                s_toast_timer_ms -= 50 ;
            } 
            else 
            {
                s_toast_timer_ms = 0 ;
                s_toast_msg = NULL ; // 时间到，清空消息
            }
        }

        // 50ms 到了，或者有按键触发了，立刻刷一次屏
        UI_Render_Current_Page() ; 
    }
}

void Task_UI_Init(void)
{
    // 优先级设为 3 
    xTaskCreate(Task_UI , "Task_UI" , 384 , NULL , 3 , &UI_taskHandle) ; 
}
