#ifndef __UI_MENU_LAYOUT_H
#define __UI_MENU_LAYOUT_H

#include "stdint.h"
#include "string.h"


typedef struct MenuPage MenuPage_t ;

typedef enum {
    PAGE_TYPE_STATUS,
    PAGE_TYPE_MENU
} MenuPageType_e ;

// 定义条目的类型
typedef enum {
    ITEM_TYPE_SUBMENU, // 子菜单入口
    ITEM_TYPE_ACTION   // 具体的功能执行项
} MenuItemType_e ;

// 定义一个带参数的动作函数指针类型
typedef void (*MenuActionCb_t)(int32_t) ;

// 定义一个菜单条目
typedef struct {
    const char *name ;          // 屏幕上显示的这行字
    MenuItemType_e type ;       // 是菜单还是动作
    union 
    {
        MenuPage_t *submenu ;   // 如果是 SUBMENU，指向下一页的指针
        struct 
        {
            MenuActionCb_t callback ; // 如果是 ACTION，指向要执行的函数指针
            int32_t        arg ;      // 传递给该函数的具体参数
        } action ;
    } payload ;
} MenuItem_t ;

// 定义一个完整的菜单页面
struct MenuPage {
    const char *title ;         // 页面的大标题 
    MenuPageType_e page_type ;  // 页面类型
    uint8_t item_count ;        // 这个页面有几个条目
    MenuItem_t *items ;         // 指向条目数组的指针
    MenuPage_t *parent ;        // 指向父页面的指针 
} ;

extern MenuPage_t page_status ;
extern MenuPage_t page_root ;

void UI_Menu_Init(void) ; 

#endif
