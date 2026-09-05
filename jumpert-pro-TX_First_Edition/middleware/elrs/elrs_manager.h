#ifndef __ELRS_MANAGER_H
#define __ELRS_MANAGER_H

#include "stdio.h"
#include "app_core/system_config_types.h"

void CRSF_Send_Ping(void) ; 
void CRSF_Request_Parameter(uint8_t param_id) ; 

void ELRS_Set_Param(ELRS_Param_ID_e param_id, uint8_t value_index) ; 
void ELRS_Trigger_Command(ELRS_Param_ID_e cmd_id) ;

#endif
