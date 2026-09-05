#include "app_task/app_task_ahrs.h"
#include "stm32f4xx_ll_exti.h"
#include "sys_hardware_config.h"
#include "utils/madgwick/Madgwick_Fusion.h"
#include "utils/mixer/mixer.h"
#include "utils/attitude/attitude.h"
#include "utils/altitude/altitude.h"
#include "utils/position/position.h"
#include "dshot_hw/ll_dshot_driver.h"
#include "dshot/dshot_protocol.h"
#include "tim/tim5.h"
#include "app_core/app_motor_unlocker.h"
#include "app_core/app_sensor_data_center.h"
#include "app_core/app_flight_mode.h"
#include "crsf/CRSF_Parser.h"
#include "vib_log/vib_log.h"
#include "pid_trace/pid_trace.h"

static const MadgwickFusion_State_t* fusion_data ;
static MIXER_INPUT mix_in ;
static MIXER_OUTPUT mix_out;
static crsf_channels_t channel ;
/* 解锁态 DShot 分频：PID 4kHz，混控+DShot 每 4 次取最新 PID 结果 -> 1kHz */
#define DSHOT_ARMED_DIVIDER  1U
static uint8_t s_dshot_armed_div = 0U; 

/* 偏航杆满量程相对航向偏置（度）；首次油门 > 70% 时捕获的航向为 0° */
#define YAW_STICK_MAX_ANGLE_DEG              90.0f
#define YAW_HEADING_CAPTURE_THROTTLE_NORM    0.70f
#define CRSF_STICK_CENTER_RAW                992U
#define CRSF_STICK_MIN_RAW                   172U
#define CRSF_STICK_MAX_RAW                   1811U
#define YAW_STICK_DEADBAND_NORM               0.05f

static float s_yaw_heading_ref_deg = 0.0f;
static uint8_t s_yaw_heading_ref_valid = 0U;
static uint8_t s_yaw_heading_capture_done = 0U;
static uint8_t s_armed_prev = 0U;

MIXER_INPUT* test(void)
{
    return &mix_in ; 
}

void task_ahrs_init(void)
{
    fusion_data = MadgwickFusion_GetStatePtr() ; 
    PID_Attitude_Init() ; 
}

void ahrs_task_trigger(void)
{
    NVIC_SetPendingIRQ(HW_IRQ_AHRS_TASK) ;
}

// 油门 -> 混控 [0, 1]
static float crsf_throttle_norm(uint16_t raw) 
{
    if (raw < 172) raw = 172;
    if (raw > 1811) raw = 1811;
    return (float)(raw - 172) / (1811.0f - 172.0f);
}

// 摇杆 -> 角度目标（度），中位 0，满杆 ±max_angle
static float crsf_stick_angle_deg(uint16_t raw, float max_angle) 
{
    float norm = (float)((int16_t)raw - (int16_t)CRSF_STICK_CENTER_RAW) / (1811.0f - 172.0f);  // 992≈中位
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f; 
    return norm * max_angle;
}

/* ch3 偏航杆 -> 相对捕获航向的偏置（度），中位 0，满杆约 ±YAW_STICK_MAX_ANGLE_DEG */
static float crsf_yaw_stick_offset_deg(uint16_t raw)
{
    float half_span = (CRSF_STICK_MAX_RAW - CRSF_STICK_MIN_RAW) * 0.5f;
    float norm = (float)((int16_t)raw - (int16_t)CRSF_STICK_CENTER_RAW) / half_span;

    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    if (norm > -YAW_STICK_DEADBAND_NORM && norm < YAW_STICK_DEADBAND_NORM) {
        norm = 0.0f;
    }

    return norm * YAW_STICK_MAX_ANGLE_DEG;
}

static void yaw_heading_capture_now(void)
{
    s_yaw_heading_ref_deg = fusion_data->euler.yaw;
    s_yaw_heading_ref_valid = 1U;
    s_yaw_heading_capture_done = 1U;
}

/* 首次油门超过阈值时捕获当前 yaw 为相对 0°（每轮解锁仅一次） */
static void yaw_heading_try_capture_on_throttle(float throttle_norm)
{
    if (s_yaw_heading_capture_done) {
        return;
    }

    if (!fusion_data->is_initialized) {
        return;
    }

    if (throttle_norm < YAW_HEADING_CAPTURE_THROTTLE_NORM) {
        return;
    }

    yaw_heading_capture_now();
}

static void yaw_heading_reset_on_disarm(void)
{
    s_yaw_heading_ref_valid = 0U;
    s_yaw_heading_ref_deg = 0.0f;
    s_yaw_heading_capture_done = 0U;
}

static float yaw_heading_target_deg(uint16_t yaw_stick_raw)
{
    float stick_offset = crsf_yaw_stick_offset_deg(yaw_stick_raw);

    if (!s_yaw_heading_ref_valid) {
        return fusion_data->euler.yaw;
    }

    return s_yaw_heading_ref_deg + stick_offset;
}

static float throttle_from_flight_mode(uint16_t raw)
{
    switch (FlightMode_Get()) 
    {
    case FLIGHT_MODE_ALT_HOLD:
    case FLIGHT_MODE_STATIC:
        return PID_AltHold_GetThrottle();

    case FLIGHT_MODE_MANUAL:
        return crsf_throttle_norm(raw);

    default:
        return crsf_throttle_norm(raw);
    }
}

static void roll_pitch_target_deg(float *roll_deg, float *pitch_deg)
{
    if (FlightMode_Get() == FLIGHT_MODE_STATIC && PID_PosHold_IsActive()) {
        *roll_deg = PID_PosHold_GetRollAngle();
        *pitch_deg = PID_PosHold_GetPitchAngle();
        return;
    }

    *pitch_deg = -crsf_stick_angle_deg(channel.ch1, 60.0f);
    *roll_deg = crsf_stick_angle_deg(channel.ch0, 60.0f);
}

static void action_regulate_pid(void)
{
    const IMU_Scaled_t *imu = imu_get_gyro_rate();
    float roll_target_deg = 0.0f;
    float pitch_target_deg = 0.0f;

    if (!fusion_data->is_initialized || imu == NULL) return;

    mix_in.throttle = throttle_from_flight_mode(channel.ch2);
    yaw_heading_try_capture_on_throttle(mix_in.throttle);
    roll_pitch_target_deg(&roll_target_deg, &pitch_target_deg);

    mix_in.pitch = PID_Attitude_Pitch(pitch_target_deg, fusion_data->euler.pitch, imu->gy, fusion_data->dt);
    mix_in.roll  = PID_Attitude_Roll(roll_target_deg, fusion_data->euler.roll,  imu->gx, fusion_data->dt);
    mix_in.yaw   = PID_Attitude_Yaw(
        yaw_heading_target_deg(channel.ch3),
        fusion_data->euler.yaw,
        imu->gz,
        fusion_data->dt);
}

static void action_regulate_mixer_dshot(void)
{
    Mixer_Update(&mix_in, &mix_out);

    dshot_trigger
    (
        DShot_Encode_Packet(mix_out.motor[0], false),
        DShot_Encode_Packet(mix_out.motor[1], false),
        DShot_Encode_Packet(mix_out.motor[2], false),
        DShot_Encode_Packet(mix_out.motor[3], false)
    );
}

static void pid_trace_feed_snapshot(uint8_t armed)
{
    const IMU_Scaled_t *imu = imu_get_gyro_rate();
    const float throttle_stick = crsf_throttle_norm(channel.ch2);
    PidTrace_Sample_t trace;

    if (!fusion_data->is_initialized || imu == NULL) {
        return;
    }

    trace.gx_deg_s = imu->gx;
    trace.gy_deg_s = imu->gy;
    trace.gz_deg_s = imu->gz;
    trace.roll_deg = fusion_data->euler.roll;
    trace.pitch_deg = fusion_data->euler.pitch;
    trace.throttle_norm = throttle_stick;
    trace.armed = armed;
    trace.ts_us = TIM5_GetStamp();

    if (armed) {
        trace.mix_roll = mix_in.roll;
        trace.mix_pitch = mix_in.pitch;
        trace.mix_yaw = mix_in.yaw;
        trace.motor[0] = mix_out.motor[0];
        trace.motor[1] = mix_out.motor[1];
        trace.motor[2] = mix_out.motor[2];
        trace.motor[3] = mix_out.motor[3];
    } else {
        trace.mix_roll = 0.0f;
        trace.mix_pitch = 0.0f;
        trace.mix_yaw = 0.0f;
        trace.motor[0] = 0.0f;
        trace.motor[1] = 0.0f;
        trace.motor[2] = 0.0f;
        trace.motor[3] = 0.0f;
    }

    PidTrace_Feed(&trace);
}

void EXTI2_IRQHandler(void) 
{
    EXTI->PR = HW_EXTIAHRS_TASK_LINE ; 
    Safety_RC_Is_Disconnected() ; 
    CRSF_ReadChannels_Control(&channel) ; 
    MadgwickFusion_UpdateWithInterpolation();
    BaroCalibrator_RefreshNav(baro_calib_get_instance());

    {
         const uint8_t armed = Safety_CanArm() ? 1U : 0U;

    if (armed) 
    {
        action_regulate_pid();

        s_dshot_armed_div++;
        if (s_dshot_armed_div >= DSHOT_ARMED_DIVIDER) {
            s_dshot_armed_div = 0U;
            action_regulate_mixer_dshot();
        }

        VibLog_SetMeta((uint8_t)(mix_in.throttle * 255.0f), 1U);
    } 
    else 
    {
        s_dshot_armed_div = 0U;

        if (s_armed_prev) 
        {
            yaw_heading_reset_on_disarm();
        }
        PID_Attitude_Reset() ; 
        PID_AltHold_Reset() ;
        PID_PosHold_Reset() ;
        dshot_trigger
        (
            DShot_Encode_Packet(0, false), 
            DShot_Encode_Packet(0, false),
            DShot_Encode_Packet(0, false),
            DShot_Encode_Packet(0, false)
        );
        VibLog_SetMeta((uint8_t)(crsf_throttle_norm(channel.ch2) * 255.0f), 0U);
    }

    pid_trace_feed_snapshot(armed);
    s_armed_prev = armed ; 
    }  

   
}
