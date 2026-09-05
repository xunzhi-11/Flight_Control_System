#ifndef __APP_SYSTEM_DATA_CENTER_H
#define __APP_SYSTEM_DATA_CENTER_H

#include "data_structure_types.h"

void gps_init(void);
void gps_get_snapshot(atgm336h_data_t *out);
void gps_apply_gga(const atgm336h_data_t *partial);
void gps_apply_rmc(const atgm336h_data_t *partial);
void gps_to_crsf_telemetry(const atgm336h_data_t *src, crsf_telemetry_gps_t *dst);

#endif
