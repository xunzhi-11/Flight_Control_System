#ifndef __BARO_OFFSET_H
#define __BARO_OFFSET_H

#include "data_structure_types.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum {
    BARO_CALIB_WAIT_WARMUP = 0,
    BARO_CALIB_COLLECTING,
    BARO_CALIB_SUCCESS
} BaroCalibState_e;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    float vertical_velocity_mps;
    uint32_t timestamp;
    float dt;
    uint8_t valid;
} BaroScaled_t;

typedef struct {
    BaroCalibState_e state;
    uint32_t sample_count;
    uint32_t last_timestamp;
    double sum_press;
    double sq_sum_press;
    BaroScaled_t output;
} BaroCalibrator_t;

void BaroCalibrator_Init(BaroCalibrator_t *calib);
bool BaroCalibrator_Update(BaroCalibrator_t *calib, const BAROMETER_Real_t *raw);
bool BaroCalibrator_IsReady(const BaroCalibrator_t *calib);
void BaroCalibrator_RefreshNav(BaroCalibrator_t *calib);
const BaroScaled_t *BaroCalibrator_GetOutput(const BaroCalibrator_t *calib);

#endif
