#ifndef __CRSF_TELEMETRY_H
#define __CRSF_TELEMETRY_H

#include "data_structure_types.h"
#include "stdint.h"


void CRSF_Send_Battery(const crsf_telemetry_battery_t* batt_data);
void CRSF_Send_GPS(const crsf_telemetry_gps_t* gps_data);
void CRSF_Send_Attitude(const crsf_telemetry_attitude_t* att_data);


#endif
