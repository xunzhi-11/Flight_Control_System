#ifndef __APP_SENSOR_DATA_CENTER_H
#define __APP_SENSOR_DATA_CENTER_H

#include "utils/imu_offset/imu_offset.h"
#include "utils/baro_offset/baro_offset.h"
#include "utils/algorithm/imu_scale.h"
#include "filter/notch_filter.h"
#include "filter/second_order_butterworth_filter.h"



/* IMU 采样与滤波默认参数（ICM42688 @ 4kHz） */
#define IMU_FILTER_FS_HZ               4065.0f
#define IMU_FILTER_GYRO_NOTCH1_F0_HZ      81.4f
#define IMU_FILTER_GYRO_NOTCH2_F0_HZ     185.0f
#define IMU_FILTER_GYRO_NOTCH_Q            5.0f
#define IMU_FILTER_GYRO_RATE_FC_HZ      225.0f
#define IMU_FILTER_GYRO_FUSION_FC_HZ    180.0f
#define IMU_FILTER_ACC_FC_HZ            100.0f
#define IMU_FILTER_ACC_NOTCH_F0_HZ        95.0f   
#define IMU_FILTER_ACC_NOTCH_Q              5.0f

GyroCalibrator_t* gyro_claib_get_instance(void);
AccCalibrator_t* acc_calib_get_instance(void);
BaroCalibrator_t* baro_calib_get_instance(void);
const BaroScaled_t* baro_scaled_get_output(void);

/* 初始化陀螺双出口滤波器及 acc 级联用滤波器实例（陷波 + PT2） */
void IMU_Filter_Init(void);

/*
 * 每帧 IMU DRDY 后调用一次；内部完成标度、陀螺/加速度零偏、陀螺双路 PT2。
 * acc 级联由调用方在 imu_update() 内扩展（见 acc 实例 getter）。
 * 返回值：Madgwick 用融合快照（gyro_fusion + acc_fusion）。
 */
IMU_Scaled_t* imu_update(void);

/* --- 双出口只读快照（imu_update 之后有效） --- */

/* 角速度内环：Notch1 → Notch2 → PT2 @ IMU_FILTER_GYRO_RATE_FC_HZ */
const IMU_Scaled_t* imu_get_gyro_rate(void);

/* Madgwick 陀螺积分：Notch1 → Notch2 → PT2 @ IMU_FILTER_GYRO_FUSION_FC_HZ */
const IMU_Scaled_t* imu_get_gyro_fusion(void);

/* 标度 + 零偏后、acc 滤波链之前的加速度 */
const IMU_Scaled_t* imu_get_acc_scaled(void);

/* acc 滤波链输出；默认等于 acc_scaled，级联完成后为陷波→PT2 结果 */
const IMU_Scaled_t* imu_get_acc_fusion(void);

/* Madgwick 融合用：gyro_fusion + acc_fusion + timestamp/dt */
const IMU_Scaled_t* imu_get_fusion_imu(void);

NotchFilter_t* imu_gyro_notch1_get(void);
NotchFilter_t* imu_gyro_notch2_get(void);
NotchFilter_t* imu_acc_notch_get(void);
ButterworthLPF_t* imu_acc_butter_get(void);

BAROMETER_Real_t* baro_update(void);
MAGNETOMETER_Calib_t* mag_update(void);
void BMM150_Manual_Calibration_AllSystem(void);

#endif
