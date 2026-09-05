#ifndef __APP_TASK_INPUT_SCAN_H
#define __APP_TASK_INPUT_SCAN_H

#include "stdint.h"
#include "stdbool.h"
#include "5_way_key/5_way_key.h"

bool Input_Get_Action(Input_Action_e *out_action, uint32_t timeout_ms) ; 

void Task_InputScan_Init(void) ; 



#endif
