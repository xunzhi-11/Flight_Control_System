#include "imu_offset.h"
#include "app_core/app_motor_unlocker.h"
#include "stdbool.h"
#include "stddef.h"
#include "stdint.h"

#define IMU_CALIB_DISCARD_SAMPLES  1000U
#define IMU_CALIB_TOTAL_SAMPLES    8000U
/* 静止采集方差阈值（机体 FRD 标度化单位） */
#define GYRO_VAR_THRESHOLD         0.5f    /* (°/s)²，约等于原 raw 阈值 100 LSB² */
#define ACC_VAR_THRESHOLD          1e-4f   /* g² */

static void imu_calib_try_clear_arm_block(const GyroCalibrator_t *gyro,
                                          const AccCalibrator_t *acc)
{
    if (gyro->state == CALIB_SUCCESS && acc->state == CALIB_SUCCESS) {
        Safety_ClearBlock(ARM_BLOCK_CALIBRATING);
    }
}

void GyroCalibrator_Init(GyroCalibrator_t *calib)
{
    calib->state = CALIB_WAIT_STABLE;
    calib->sample_count = 0;
    calib->last_timestamp = 0;
    calib->sum_gx = calib->sum_gy = calib->sum_gz = 0;
    calib->sq_sum_gx = calib->sq_sum_gy = calib->sq_sum_gz = 0;
    calib->bias_gx = calib->bias_gy = calib->bias_gz = 0.0f;
}

bool GyroCalibrator_Update(GyroCalibrator_t *calib, const IMU_Scaled_t *imu_body,
                           const AccCalibrator_t *acc_calib)
{
    if (calib->state == CALIB_SUCCESS) {
        return true;
    }

    if (imu_body->timestamp == calib->last_timestamp) {
        return false;
    }

    calib->last_timestamp = imu_body->timestamp;

    if (calib->state == CALIB_WAIT_STABLE) {
        calib->sample_count++;
        if (calib->sample_count >= IMU_CALIB_DISCARD_SAMPLES) {
            calib->sample_count = 0;
            calib->state = CALIB_COLLECTING;
        }
        return false;
    }

    if (calib->state == CALIB_COLLECTING) {
        float gx = imu_body->gx;
        float gy = imu_body->gy;
        float gz = imu_body->gz;

        calib->sum_gx += gx;
        calib->sum_gy += gy;
        calib->sum_gz += gz;

        calib->sq_sum_gx += (gx * gx);
        calib->sq_sum_gy += (gy * gy);
        calib->sq_sum_gz += (gz * gz);

        calib->sample_count++;

        if (calib->sample_count >= IMU_CALIB_TOTAL_SAMPLES) {
            float mean_x = calib->sum_gx / (float)IMU_CALIB_TOTAL_SAMPLES;
            float mean_y = calib->sum_gy / (float)IMU_CALIB_TOTAL_SAMPLES;
            float mean_z = calib->sum_gz / (float)IMU_CALIB_TOTAL_SAMPLES;

            float var_x = (calib->sq_sum_gx / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_x * mean_x);
            float var_y = (calib->sq_sum_gy / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_y * mean_y);
            float var_z = (calib->sq_sum_gz / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_z * mean_z);

            if (var_x < GYRO_VAR_THRESHOLD &&
                var_y < GYRO_VAR_THRESHOLD &&
                var_z < GYRO_VAR_THRESHOLD)
            {
                calib->bias_gx = mean_x;
                calib->bias_gy = mean_y;
                calib->bias_gz = mean_z;
                calib->state = CALIB_SUCCESS;
                if (acc_calib != NULL) {
                    imu_calib_try_clear_arm_block(calib, acc_calib);
                }
                return true;
            }

            calib->sample_count = 0;
            calib->sum_gx = calib->sum_gy = calib->sum_gz = 0;
            calib->sq_sum_gx = calib->sq_sum_gy = calib->sq_sum_gz = 0;
        }
    }
    return false;
}

void AccCalibrator_Init(AccCalibrator_t *calib)
{
    calib->state = CALIB_WAIT_STABLE;
    calib->sample_count = 0;
    calib->last_timestamp = 0;
    calib->sum_ax = calib->sum_ay = calib->sum_az = 0;
    calib->sq_sum_ax = calib->sq_sum_ay = calib->sq_sum_az = 0;
    calib->bias_ax = calib->bias_ay = calib->bias_az = 0.0f;
}

bool AccCalibrator_Update(AccCalibrator_t *calib, const IMU_Scaled_t *imu_body,
                          const GyroCalibrator_t *gyro_calib)
{
    if (calib->state == CALIB_SUCCESS) {
        return true;
    }

    if (imu_body->timestamp == calib->last_timestamp) {
        return false;
    }

    calib->last_timestamp = imu_body->timestamp;

    if (calib->state == CALIB_WAIT_STABLE) {
        calib->sample_count++;
        if (calib->sample_count >= IMU_CALIB_DISCARD_SAMPLES) {
            calib->sample_count = 0;
            calib->state = CALIB_COLLECTING;
        }
        return false;
    }

    if (calib->state == CALIB_COLLECTING) {
        float ax = imu_body->ax;
        float ay = imu_body->ay;
        float az = imu_body->az;

        calib->sum_ax += ax;
        calib->sum_ay += ay;
        calib->sum_az += az;

        calib->sq_sum_ax += (ax * ax);
        calib->sq_sum_ay += (ay * ay);
        calib->sq_sum_az += (az * az);

        calib->sample_count++;

        if (calib->sample_count >= IMU_CALIB_TOTAL_SAMPLES) {
            float mean_x = calib->sum_ax / (float)IMU_CALIB_TOTAL_SAMPLES;
            float mean_y = calib->sum_ay / (float)IMU_CALIB_TOTAL_SAMPLES;
            float mean_z = calib->sum_az / (float)IMU_CALIB_TOTAL_SAMPLES;

            float var_x = (calib->sq_sum_ax / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_x * mean_x);
            float var_y = (calib->sq_sum_ay / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_y * mean_y);
            float var_z = (calib->sq_sum_az / (float)IMU_CALIB_TOTAL_SAMPLES) - (mean_z * mean_z);

            if (var_x < ACC_VAR_THRESHOLD &&
                var_y < ACC_VAR_THRESHOLD &&
                var_z < ACC_VAR_THRESHOLD)
            {
                calib->bias_ax = mean_x;
                calib->bias_ay = mean_y;
                calib->bias_az = mean_z - 1.0f;
                calib->state = CALIB_SUCCESS;
                if (gyro_calib != NULL) {
                    imu_calib_try_clear_arm_block(gyro_calib, calib);
                }
                return true;
            }

            calib->sample_count = 0;
            calib->sum_ax = calib->sum_ay = calib->sum_az = 0;
            calib->sq_sum_ax = calib->sq_sum_ay = calib->sq_sum_az = 0;
        }
    }
    return false;
}
