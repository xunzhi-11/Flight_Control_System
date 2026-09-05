#include "utils/baro_offset/baro_offset.h"
#include "utils/madgwick/Madgwick_Fusion.h"
#include "app_core/app_motor_unlocker.h"

#define BARO_CALIB_WARMUP_SAMPLES   50U
#define BARO_CALIB_TOTAL_SAMPLES    30U
#define BARO_PRESS_VAR_THRESHOLD    9.0f

static void baro_reset_collect(BaroCalibrator_t *calib)
{
    calib->sample_count = 0;
    calib->sum_press = 0.0;
    calib->sq_sum_press = 0.0;
}

void BaroCalibrator_Init(BaroCalibrator_t *calib)
{
    calib->state = BARO_CALIB_WAIT_WARMUP;
    calib->sample_count = 0;
    calib->last_timestamp = 0;
    baro_reset_collect(calib);

    calib->output.pressure_pa = 0.0f;
    calib->output.temperature_c = 0.0f;
    calib->output.altitude_m = 0.0f;
    calib->output.vertical_velocity_mps = 0.0f;
    calib->output.timestamp = 0;
    calib->output.dt = 0.0f;
    calib->output.valid = 0;
}

bool BaroCalibrator_IsReady(const BaroCalibrator_t *calib)
{
    return (calib != NULL) && (calib->state == BARO_CALIB_SUCCESS);
}

void BaroCalibrator_RefreshNav(BaroCalibrator_t *calib)
{
    if (calib == NULL) {
        return;
    }

    const MadgwickFusion_State_t *fusion = MadgwickFusion_GetStatePtr();
    if (fusion == NULL || !fusion->baro_valid) {
        calib->output.valid = 0;
        return;
    }

    calib->output.altitude_m = fusion->altitude_state.altitude;
    calib->output.vertical_velocity_mps = fusion->altitude_state.vertical_velocity;
    calib->output.valid = 1;
}

const BaroScaled_t *BaroCalibrator_GetOutput(const BaroCalibrator_t *calib)
{
    if (calib == NULL) {
        return NULL;
    }

    return &calib->output;
}

bool BaroCalibrator_Update(BaroCalibrator_t *calib, const BAROMETER_Real_t *raw)
{
    if (calib == NULL || raw == NULL) {
        return false;
    }

    if (calib->state == BARO_CALIB_SUCCESS) {
        if (raw->god_time != calib->last_timestamp) {
            calib->last_timestamp = raw->god_time;
            calib->output.pressure_pa = (float)raw->press;
            calib->output.temperature_c = (float)raw->temp;
            calib->output.timestamp = raw->god_time;
            calib->output.dt = raw->dt;
        }
        return true;
    }

    if (raw->god_time == calib->last_timestamp) {
        return false;
    }

    calib->last_timestamp = raw->god_time;
    calib->output.pressure_pa = (float)raw->press;
    calib->output.temperature_c = (float)raw->temp;
    calib->output.timestamp = raw->god_time;
    calib->output.dt = raw->dt;

    if (raw->press <= 0.0) {
        return false;
    }

    if (calib->state == BARO_CALIB_WAIT_WARMUP) {
        calib->sample_count++;
        if (calib->sample_count >= BARO_CALIB_WARMUP_SAMPLES) {
            baro_reset_collect(calib);
            calib->state = BARO_CALIB_COLLECTING;
        }
        return false;
    }

    if (calib->state == BARO_CALIB_COLLECTING) {
        double press = raw->press;

        calib->sum_press += press;
        calib->sq_sum_press += press * press;
        calib->sample_count++;

        if (calib->sample_count >= BARO_CALIB_TOTAL_SAMPLES) {
            float mean = (float)(calib->sum_press / (double)BARO_CALIB_TOTAL_SAMPLES);
            float mean_sq = (float)(calib->sq_sum_press / (double)BARO_CALIB_TOTAL_SAMPLES);
            float var = mean_sq - mean * mean;

            if (var < BARO_PRESS_VAR_THRESHOLD) {
                MadgwickFusion_CalibrateAltitudeAtPressure(mean, (float)raw->temp);
                calib->state = BARO_CALIB_SUCCESS;
                Safety_ClearBlock(ARM_BLOCK_BARO_CALIBRATING);
                return true;
            }

            baro_reset_collect(calib);
        }
    }

    return false;
}
