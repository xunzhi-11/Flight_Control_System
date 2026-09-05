#ifndef __JOY_CALIB_FLOW_H
#define __JOY_CALIB_FLOW_H

#include "stdbool.h"
#include "5_way_key/5_way_key.h"

bool JoyCalib_Is_Active(void) ;
void JoyCalib_Start(void) ;
void JoyCalib_Process(Input_Action_e action) ;
void JoyCalib_Tick(void) ;
void JoyCalib_Render(void) ;

#endif
