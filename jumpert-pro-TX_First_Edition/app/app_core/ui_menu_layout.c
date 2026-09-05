#include "app_core/ui_menu_layout.h"
#include "Application/ui_actions.h"
#include "app_core/system_state.h"
//  定义底层子菜单 (叶子节点) 
// 高频头设置页面的条目

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

MenuItem_t packet_rate_items[] = {
    {"50Hz",  ITEM_TYPE_ACTION,  {.action = {.callback = Action_Set_PacketRate, .arg = PKT_RATE_50HZ}}},
    {"100Hz", ITEM_TYPE_ACTION,  {.action = {.callback = Action_Set_PacketRate, .arg = PKT_RATE_100HZ}}},
    {"150Hz", ITEM_TYPE_ACTION,  {.action = {.callback = Action_Set_PacketRate, .arg = PKT_RATE_150HZ}}},
    {"250Hz", ITEM_TYPE_ACTION,  {.action = {.callback = Action_Set_PacketRate, .arg = PKT_RATE_250HZ}}},
    {"333Hz", ITEM_TYPE_ACTION,  {.action = {.callback = Action_Set_PacketRate, .arg = PKT_RATE_333HZ}}}
} ;
MenuPage_t page_packet_rate = {"PACKET RATE", PAGE_TYPE_MENU, ARRAY_SIZE(packet_rate_items), packet_rate_items, NULL} ;

MenuItem_t tx_max_power_items[] = {
    {"100mw" , ITEM_TYPE_ACTION , {.action = {.callback = Action_Set_TX_Power, .arg = PWR_100MW}}} , 
    {"10mw" , ITEM_TYPE_ACTION , {.action = {.callback = Action_Set_TX_Power, .arg = PWR_10MW}}} 
} ; 
MenuPage_t page_tx_power = {"TX MAX_POWER" , PAGE_TYPE_MENU, ARRAY_SIZE(tx_max_power_items) , tx_max_power_items , NULL} ; 

MenuItem_t telem_ratio_items[] = {
    {"Std",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_DEFAULT}}},
    {"Off",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_OFF}}},
    {"1:128", ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_128}}},
    {"1:64",  ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_64}}},
    {"1:32",  ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_32}}},
    {"1:16",  ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_16}}},
    {"1:8",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_8}}},
    {"1:4",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_4}}},
    {"1:2",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_2}}},
    {"1:1",   ITEM_TYPE_ACTION, {.action = {.callback = Action_Set_TelemRatio, .arg = TELEM_RATIO_1_1}}}
} ;
MenuPage_t page_telem_ratio = {"TELEM RATIO", PAGE_TYPE_MENU, ARRAY_SIZE(telem_ratio_items), telem_ratio_items, NULL} ;

MenuItem_t tuner_items[] = {
    {"Packet Rate", ITEM_TYPE_SUBMENU,  {.submenu = &page_packet_rate}},
    {"TX Power",    ITEM_TYPE_SUBMENU,  {.submenu = &page_tx_power}},
    {"Telem Ratio", ITEM_TYPE_SUBMENU, {.submenu = &page_telem_ratio}},
    {"Bind",        ITEM_TYPE_ACTION,  {.action = {.callback = Action_ELRS_Bind, .arg = 0}}}
} ;
MenuPage_t page_tuner = {"ELRS CONFIG", PAGE_TYPE_MENU, ARRAY_SIZE(tuner_items), tuner_items, NULL} ; // parent 稍后绑定

// 系统设置页面的条目
MenuItem_t sys_setup_items[] = {
    {"Joy Calib",   ITEM_TYPE_ACTION,  {.action = {.callback = Action_Start_Joy_Calib, .arg = 0}}},
    {"Backlight",   ITEM_TYPE_ACTION,  {.action = {.callback = NULL, .arg = 0}}}
} ;
MenuPage_t page_sys_setup = {"SYS SETUP", PAGE_TYPE_MENU, ARRAY_SIZE(sys_setup_items), sys_setup_items, NULL} ;

//  定义根菜单
MenuItem_t root_items[] = {
    {"System Setup", ITEM_TYPE_SUBMENU, {.submenu = &page_sys_setup}},
    {"Tuner Setup",  ITEM_TYPE_SUBMENU, {.submenu = &page_tuner}}
} ;
MenuPage_t page_root = {"MAIN MENU", PAGE_TYPE_MENU, ARRAY_SIZE(root_items), root_items, NULL} ;

MenuItem_t status_items[] = {
    {"Main Menu", ITEM_TYPE_SUBMENU, {.submenu = &page_root}}
} ;
MenuPage_t page_status = {"STATUS", PAGE_TYPE_STATUS, ARRAY_SIZE(status_items), status_items, NULL} ;


//  绑定父子关系 
void UI_Menu_Init(void) 
{
    page_root.parent = &page_status ;
    page_tuner.parent = &page_root ;
    page_sys_setup.parent = &page_root ;
    page_packet_rate.parent = &page_tuner ;
    page_tx_power.parent = &page_tuner ;
    page_telem_ratio.parent = &page_tuner ;
}


