#ifndef __IMU_OFFSET_H
#define __IMU_OFFSET_H

#include "data_structure_types.h"
#include "stdint.h"
#include "stdbool.h"

typedef enum 
{
    CALIB_WAIT_STABLE = 0,
    CALIB_COLLECTING,
    CALIB_SUCCESS
} CalibState_e ;

typedef struct {
    CalibState_e state ;
    uint32_t     sample_count ;
    uint32_t     last_timestamp ;     // 用于校验数据是否更新
    
    float        sum_gx, sum_gy, sum_gz ;
    float        sq_sum_gx, sq_sum_gy, sq_sum_gz ;

    float        bias_gx, bias_gy, bias_gz ;       // 机体 FRD 零偏 (°/s)
} GyroCalibrator_t ;

typedef struct {
    CalibState_e state ;
    uint32_t     sample_count ;
    uint32_t     last_timestamp ;

    float        sum_ax, sum_ay, sum_az ;
    float        sq_sum_ax, sq_sum_ay, sq_sum_az ;

    float        bias_ax, bias_ay, bias_az ;       // 机体 FRD，相对 [0,0,+1]g 的零偏 (g)
} AccCalibrator_t ;


void GyroCalibrator_Init(GyroCalibrator_t *calib) ;
bool GyroCalibrator_Update(GyroCalibrator_t *calib, const IMU_Scaled_t *imu_body,
                          const AccCalibrator_t *acc_calib) ;

void AccCalibrator_Init(AccCalibrator_t *calib) ;
bool AccCalibrator_Update(AccCalibrator_t *calib, const IMU_Scaled_t *imu_body,
                          const GyroCalibrator_t *gyro_calib) ;


#endif

