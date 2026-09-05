#ifndef __UI_ACTIONS_H
#define __UI_ACTIONS_H

#include "stdint.h"

void Action_ELRS_Bind(int32_t arg) ; 
void Action_Start_Joy_Calib(int32_t arg) ; 
void Action_Set_TX_Power(int32_t power) ; 
void Action_Set_PacketRate(int32_t rate)  ;
void Action_Set_TelemRatio(int32_t ratio) ; 
void UI_Show_Toast(char* msg) ; 

#endif