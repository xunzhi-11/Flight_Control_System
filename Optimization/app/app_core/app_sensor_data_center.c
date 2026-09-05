#include "app_core/app_sensor_data_center.h"
#include "icm42688/icm42688.h"
#include "utils/imu_offset/imu_offset.h"
#include "utils/baro_offset/baro_offset.h"
#include "utils/algorithm/imu_scale.h"
#include "utils/algorithm/mag_calibration.h"
#include "bmp390l/bmp390l.h"
#include "bmm150/bmm150.h"
#include "tim/tim5.h"
#include "vib_log/vib_log.h"

static GyroCalibrator_t gyro_calibrator;
static AccCalibrator_t acc_calibrator;
static BaroCalibrator_t baro_calibrator;
static MagCal_Param_t mag_calib_param;

static NotchFilter_t s_gyro_notch1;
static NotchFilter_t s_gyro_notch2;
static ButterworthLPF_t s_gyro_rate_lpf;
static ButterworthLPF_t s_gyro_fusion_lpf;
static NotchFilter_t s_acc_notch;
static ButterworthLPF_t s_acc_butter_lpf;

static IMU_Raw_t imu_raw;
static BAROMETER_Real_t baro_real;
static MAGNETOMETER_Trimed_t mag_trimed;
static MAGNETOMETER_Calib_t mag_calib;
static IMU_Scaled_t imu_scaled;

static IMU_Scaled_t s_gyro_rate_out;
static IMU_Scaled_t s_gyro_fusion_out;
static IMU_Scaled_t s_acc_fusion_out;
static IMU_Scaled_t s_fusion_imu_out;

static uint32_t s_imu_last_event_ts;
static uint32_t s_baro_last_event_ts;
static uint32_t s_mag_last_event_ts;

static void imu_remap_to_body_frd(IMU_Scaled_t *imu)
{
    // imu->ay = - imu->ay ; 
    // imu->az = - imu->az ;
    imu->ax = - imu->ax ;  
    imu->gy = - imu->gy ;
    imu->gz = - imu->gz ;
}

static void mag_remap_to_body_frd(const float mag_sensor[3], float mag_body[3])
{
    mag_body[0] = mag_sensor[1];
    mag_body[1] = -mag_sensor[0];
    mag_body[2] = -mag_sensor[2];
}

static void imu_copy_meta(IMU_Scaled_t *dst, const IMU_Scaled_t *src)
{
    dst->timestamp = src->timestamp;
    dst->dt = src->dt;
}

static void imu_copy_gyro(IMU_Scaled_t *dst, const IMU_Scaled_t *src)
{
    dst->gx = src->gx;
    dst->gy = src->gy;
    dst->gz = src->gz;
}

static void imu_copy_acc(IMU_Scaled_t *dst, const IMU_Scaled_t *src)
{
    dst->ax = src->ax;
    dst->ay = src->ay;
    dst->az = src->az;
}

static void imu_build_fusion_snapshot(void)
{
    imu_copy_gyro(&s_fusion_imu_out, &s_gyro_fusion_out);
    imu_copy_acc(&s_fusion_imu_out, &s_acc_fusion_out);
    imu_copy_meta(&s_fusion_imu_out, &imu_scaled);
}


static void imu_apply_acc_filter_chain(void)
{
    IMU_NotchFilter_Update(&s_acc_notch, &imu_scaled);
    IMU_ButterworthLPF_Update(&s_acc_butter_lpf, &s_acc_notch.out);
    imu_copy_acc(&s_acc_fusion_out, &s_acc_butter_lpf.out);
    imu_copy_meta(&s_acc_fusion_out, &imu_scaled);
}

static void imu_apply_gyro_dual_filters(void)
{
    IMU_NotchFilter_Update(&s_gyro_notch1, &imu_scaled);
    IMU_NotchFilter_Update(&s_gyro_notch2, &s_gyro_notch1.out);
    IMU_ButterworthLPF_Update(&s_gyro_rate_lpf, &s_gyro_notch2.out);
    IMU_ButterworthLPF_Update(&s_gyro_fusion_lpf, &s_gyro_notch2.out);

    imu_copy_gyro(&s_gyro_rate_out, &s_gyro_rate_lpf.out);
    imu_copy_meta(&s_gyro_rate_out, &imu_scaled);

    imu_copy_gyro(&s_gyro_fusion_out, &s_gyro_fusion_lpf.out);
    imu_copy_meta(&s_gyro_fusion_out, &imu_scaled);
}

GyroCalibrator_t* gyro_claib_get_instance(void) { return &gyro_calibrator; }
AccCalibrator_t* acc_calib_get_instance(void) { return &acc_calibrator; }
BaroCalibrator_t* baro_calib_get_instance(void) { return &baro_calibrator; }

void IMU_Filter_Init(void)
{
    const float fs = IMU_FILTER_FS_HZ;

    IMU_NotchFilter_Init(&s_gyro_notch1,
                         0.0f,
                         IMU_FILTER_GYRO_NOTCH1_F0_HZ,
                         IMU_FILTER_GYRO_NOTCH_Q,
                         fs);
    IMU_NotchFilter_Init(&s_gyro_notch2,
                         0.0f,
                         IMU_FILTER_GYRO_NOTCH2_F0_HZ,
                         IMU_FILTER_GYRO_NOTCH_Q,
                         fs);
    IMU_ButterworthLPF_Init(&s_gyro_rate_lpf, 0.0f, IMU_FILTER_GYRO_RATE_FC_HZ, fs);
    IMU_ButterworthLPF_Init(&s_gyro_fusion_lpf, 0.0f, IMU_FILTER_GYRO_FUSION_FC_HZ, fs);

    IMU_NotchFilter_Init(&s_acc_notch,
                         IMU_FILTER_ACC_NOTCH_F0_HZ,
                         0.0f,
                         IMU_FILTER_ACC_NOTCH_Q,
                         fs);
    IMU_ButterworthLPF_Init(&s_acc_butter_lpf, IMU_FILTER_ACC_FC_HZ, 0.0f, fs);
}

NotchFilter_t* imu_gyro_notch1_get(void) { return &s_gyro_notch1; }
NotchFilter_t* imu_gyro_notch2_get(void) { return &s_gyro_notch2; }
NotchFilter_t* imu_acc_notch_get(void) { return &s_acc_notch; }
ButterworthLPF_t* imu_acc_butter_get(void) { return &s_acc_butter_lpf; }

const IMU_Scaled_t* imu_get_gyro_rate(void) { return &s_gyro_rate_out; }
const IMU_Scaled_t* imu_get_gyro_fusion(void) { return &s_gyro_fusion_out; }
const IMU_Scaled_t* imu_get_acc_scaled(void) { return &imu_scaled; }
const IMU_Scaled_t* imu_get_acc_fusion(void) { return &s_acc_fusion_out; }
const IMU_Scaled_t* imu_get_fusion_imu(void) { return &s_fusion_imu_out; }

const BaroScaled_t* baro_scaled_get_output(void)
{
    return BaroCalibrator_GetOutput(&baro_calibrator);
}

IMU_Scaled_t* imu_update(void)
{
    icm42688_get_rawdata(&imu_raw);

    if (imu_raw.irq_timestamp == s_imu_last_event_ts) {
        return &s_fusion_imu_out;
    }

    imu_scaled.ax = acc_convert(imu_raw.ax);
    imu_scaled.ay = acc_convert(imu_raw.ay);
    imu_scaled.az = acc_convert(imu_raw.az);
    imu_scaled.gx = gyro_convert(imu_raw.gx);
    imu_scaled.gy = gyro_convert(imu_raw.gy);
    imu_scaled.gz = gyro_convert(imu_raw.gz);
    imu_scaled.timestamp = imu_raw.irq_timestamp;
    imu_scaled.dt = TIM5_DtFromEventStamp(imu_raw.irq_timestamp, &s_imu_last_event_ts);

    imu_remap_to_body_frd(&imu_scaled);

    GyroCalibrator_Update(&gyro_calibrator, &imu_scaled, &acc_calibrator);
    AccCalibrator_Update(&acc_calibrator, &imu_scaled, &gyro_calibrator);

    if (acc_calibrator.state == CALIB_SUCCESS) {
        imu_scaled.ax -= acc_calibrator.bias_ax;
        imu_scaled.ay -= acc_calibrator.bias_ay;
        imu_scaled.az -= acc_calibrator.bias_az;
    }

    if (gyro_calibrator.state == CALIB_SUCCESS) {
        imu_scaled.gx -= gyro_calibrator.bias_gx;
        imu_scaled.gy -= gyro_calibrator.bias_gy;
        imu_scaled.gz -= gyro_calibrator.bias_gz;
    }

    VibLog_FeedImu(imu_scaled.gx, imu_scaled.gy, imu_scaled.gz, imu_scaled.timestamp);

    imu_apply_gyro_dual_filters();
    imu_apply_acc_filter_chain();
    imu_build_fusion_snapshot();

    return &s_fusion_imu_out;
}

BAROMETER_Real_t* baro_update(void)
{
    bmp390l_get_real_data(&baro_real);

    if (baro_real.god_time == s_baro_last_event_ts) {
        return &baro_real;
    }

    baro_real.dt = TIM5_DtFromEventStamp(baro_real.god_time, &s_baro_last_event_ts);
    BaroCalibrator_Update(&baro_calibrator, &baro_real);
    return &baro_real;
}

MAGNETOMETER_Calib_t* mag_update(void)
{
    bmm150_get_trimed_data(&mag_trimed);

    if (mag_trimed.god_time == s_mag_last_event_ts) {
        return &mag_calib;
    }

    float mag_in[3] = {mag_trimed.mag_x, mag_trimed.mag_y, mag_trimed.mag_z};
    float mag_out[3] = {0.0f};
    MagCal_Apply(mag_in, &mag_calib_param, mag_out);

    float mag_body[3];
    mag_remap_to_body_frd(mag_out, mag_body);

    mag_calib.god_time = mag_trimed.god_time;
    mag_calib.dt = TIM5_DtFromEventStamp(mag_trimed.god_time, &s_mag_last_event_ts);
    mag_calib.mag_x = mag_body[0];
    mag_calib.mag_y = mag_body[1];
    mag_calib.mag_z = mag_body[2];

    return &mag_calib;
}

void BMM150_Manual_Calibration_AllSystem(void)
{
    mag_calib_param.hard_iron_offset[0] = -10.3779f;
    mag_calib_param.hard_iron_offset[1] = -0.9265f;
    mag_calib_param.hard_iron_offset[2] = -3.3946f;

    mag_calib_param.soft_iron_matrix[0][0] = 1.0153f;
    mag_calib_param.soft_iron_matrix[0][1] = 0.0287f;
    mag_calib_param.soft_iron_matrix[0][2] = 0.0330f;

    mag_calib_param.soft_iron_matrix[1][0] = 0.0287f;
    mag_calib_param.soft_iron_matrix[1][1] = 1.0307f;
    mag_calib_param.soft_iron_matrix[1][2] = -0.0244f;

    mag_calib_param.soft_iron_matrix[2][0] = 0.0330f;
    mag_calib_param.soft_iron_matrix[2][1] = -0.0244f;
    mag_calib_param.soft_iron_matrix[2][2] = 0.9649f;
}
