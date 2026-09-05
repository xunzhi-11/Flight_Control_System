#include "utils/position/position.h"
#include "utils/pid/pid.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CRSF_CHANNEL_MIN        172U
#define CRSF_CHANNEL_MAX        1811U
#define CRSF_CHANNEL_MID        992U
#define CRSF_CHANNEL_SPAN       (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)

#define METERS_PER_DEG_LAT      111320.0
#define GPS_MIN_FIX_QUALITY     1U
#define GPS_MAX_HDOP              3.0f

#define STICK_DEADZONE          0.05f
#define MAX_POS_SPEED             2.0f
#define MAX_TILT_ANGLE             15.0f
#define VEL_LPF_ALPHA              0.35f

#define KP_POS                    0.8f
#define KI_POS                    0.0f
#define KD_POS                    0.0f
#define MAX_OUT_POS                 2.0f
#define MAX_OUT_I_POS               0.0f

#define KP_VEL                    8.0f
#define KI_VEL                    0.2f
#define KD_VEL                    0.05f
#define MAX_OUT_VEL                MAX_TILT_ANGLE
#define MAX_OUT_I_VEL               5.0f

typedef struct {
    PID_Controller pid_pos_n;
    PID_Controller pid_pos_e;
    PID_Controller pid_vel_n;
    PID_Controller pid_vel_e;

    double ref_lat;
    double ref_lon;
    float target_n;
    float target_e;

    float vel_n;
    float vel_e;
    float prev_n;
    float prev_e;
    uint8_t vel_valid;

    float roll_angle;
    float pitch_angle;
    bool enabled;
    bool active;
    bool stick_in_deadzone_prev;
} PosHold_State_t;

static PosHold_State_t s_pos_hold;

static bool gps_is_usable(const atgm336h_data_t *gps)
{
    if (gps == NULL) {
        return false;
    }

    if (!gps->is_valid || gps->fix_quality < GPS_MIN_FIX_QUALITY) {
        return false;
    }

    if (gps->hdop > GPS_MAX_HDOP) {
        return false;
    }

    return true;
}

static void latlon_to_ne(double lat, double lon, float *north_m, float *east_m)
{
    const double d_lat = lat - s_pos_hold.ref_lat;
    const double d_lon = lon - s_pos_hold.ref_lon;
    const float ref_lat_rad = (float)(s_pos_hold.ref_lat * (M_PI / 180.0));

    *north_m = (float)(d_lat * METERS_PER_DEG_LAT);
    *east_m = (float)(d_lon * METERS_PER_DEG_LAT * cosf(ref_lat_rad));
}

static void body_tilt_from_ne(float cmd_n, float cmd_e, float yaw_deg,
                              float *roll_deg, float *pitch_deg)
{
    const float yaw_rad = yaw_deg * (float)(M_PI / 180.0);
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float forward = cmd_n * cos_yaw + cmd_e * sin_yaw;
    const float right = -cmd_n * sin_yaw + cmd_e * cos_yaw;

    *pitch_deg = -forward;
    *roll_deg = right;
}

static void pos_hold_clamp_tilt(float *roll_deg, float *pitch_deg)
{
    if (*roll_deg > MAX_TILT_ANGLE) {
        *roll_deg = MAX_TILT_ANGLE;
    } else if (*roll_deg < -MAX_TILT_ANGLE) {
        *roll_deg = -MAX_TILT_ANGLE;
    }

    if (*pitch_deg > MAX_TILT_ANGLE) {
        *pitch_deg = MAX_TILT_ANGLE;
    } else if (*pitch_deg < -MAX_TILT_ANGLE) {
        *pitch_deg = -MAX_TILT_ANGLE;
    }
}

void PID_PosHold_Init(void)
{
    s_pos_hold.enabled = false;
    s_pos_hold.active = false;
    s_pos_hold.stick_in_deadzone_prev = true;
    s_pos_hold.vel_valid = 0U;
    s_pos_hold.roll_angle = 0.0f;
    s_pos_hold.pitch_angle = 0.0f;

    PID_Init(&s_pos_hold.pid_pos_n, KP_POS, KI_POS, KD_POS, MAX_OUT_POS, MAX_OUT_I_POS);
    PID_Init(&s_pos_hold.pid_pos_e, KP_POS, KI_POS, KD_POS, MAX_OUT_POS, MAX_OUT_I_POS);
    PID_Init(&s_pos_hold.pid_vel_n, KP_VEL, KI_VEL, KD_VEL, MAX_OUT_VEL, MAX_OUT_I_VEL);
    PID_Init(&s_pos_hold.pid_vel_e, KP_VEL, KI_VEL, KD_VEL, MAX_OUT_VEL, MAX_OUT_I_VEL);
}

void PID_PosHold_Reset(void)
{
    s_pos_hold.enabled = false;
    s_pos_hold.active = false;
    s_pos_hold.stick_in_deadzone_prev = true;
    s_pos_hold.vel_valid = 0U;
    s_pos_hold.roll_angle = 0.0f;
    s_pos_hold.pitch_angle = 0.0f;

    PID_Reset(&s_pos_hold.pid_pos_n);
    PID_Reset(&s_pos_hold.pid_pos_e);
    PID_Reset(&s_pos_hold.pid_vel_n);
    PID_Reset(&s_pos_hold.pid_vel_e);
}

void PID_PosHold_Enable(const atgm336h_data_t *gps)
{
    if (!gps_is_usable(gps)) {
        s_pos_hold.enabled = true;
        s_pos_hold.active = false;
        return;
    }

    s_pos_hold.ref_lat = gps->latitude;
    s_pos_hold.ref_lon = gps->longitude;
    latlon_to_ne(gps->latitude, gps->longitude, &s_pos_hold.target_n, &s_pos_hold.target_e);
    s_pos_hold.prev_n = s_pos_hold.target_n;
    s_pos_hold.prev_e = s_pos_hold.target_e;
    s_pos_hold.vel_n = 0.0f;
    s_pos_hold.vel_e = 0.0f;
    s_pos_hold.vel_valid = 0U;
    s_pos_hold.enabled = true;
    s_pos_hold.active = true;
    s_pos_hold.stick_in_deadzone_prev = true;
    s_pos_hold.roll_angle = 0.0f;
    s_pos_hold.pitch_angle = 0.0f;

    PID_Reset(&s_pos_hold.pid_pos_n);
    PID_Reset(&s_pos_hold.pid_pos_e);
    PID_Reset(&s_pos_hold.pid_vel_n);
    PID_Reset(&s_pos_hold.pid_vel_e);
}

bool PID_PosHold_IsActive(void)
{
    return s_pos_hold.active;
}

float PID_PosHold_StickNorm(uint16_t raw)
{
    float norm = (float)((int32_t)raw - (int32_t)CRSF_CHANNEL_MID) / (float)CRSF_CHANNEL_SPAN;

    if (norm > 1.0f) {
        norm = 1.0f;
    } else if (norm < -1.0f) {
        norm = -1.0f;
    }

    return norm;
}

void PID_PosHold_Update(const atgm336h_data_t *gps,
                        float roll_stick_norm,
                        float pitch_stick_norm,
                        float yaw_deg,
                        float dt)
{
    float current_n = 0.0f;
    float current_e = 0.0f;
    float target_vn = 0.0f;
    float target_ve = 0.0f;
    float cmd_n = 0.0f;
    float cmd_e = 0.0f;
    const bool roll_in_deadzone = (roll_stick_norm >= -STICK_DEADZONE &&
                                   roll_stick_norm <= STICK_DEADZONE);
    const bool pitch_in_deadzone = (pitch_stick_norm >= -STICK_DEADZONE &&
                                    pitch_stick_norm <= STICK_DEADZONE);
    const bool stick_in_deadzone = roll_in_deadzone && pitch_in_deadzone;

    if (!s_pos_hold.enabled || !gps_is_usable(gps)) {
        s_pos_hold.active = false;
        s_pos_hold.roll_angle = 0.0f;
        s_pos_hold.pitch_angle = 0.0f;
        return;
    }

    s_pos_hold.active = true;
    latlon_to_ne(gps->latitude, gps->longitude, &current_n, &current_e);

    if (s_pos_hold.vel_valid && dt > 0.0f) {
        const float vn = (current_n - s_pos_hold.prev_n) / dt;
        const float ve = (current_e - s_pos_hold.prev_e) / dt;

        s_pos_hold.vel_n += VEL_LPF_ALPHA * (vn - s_pos_hold.vel_n);
        s_pos_hold.vel_e += VEL_LPF_ALPHA * (ve - s_pos_hold.vel_e);
    } else {
        s_pos_hold.vel_valid = 1U;
    }

    s_pos_hold.prev_n = current_n;
    s_pos_hold.prev_e = current_e;

    if (!stick_in_deadzone) {
        target_vn = -pitch_stick_norm * MAX_POS_SPEED;
        target_ve = roll_stick_norm * MAX_POS_SPEED;
        s_pos_hold.target_n += target_vn * dt;
        s_pos_hold.target_e += target_ve * dt;
        s_pos_hold.stick_in_deadzone_prev = false;
    } else {
        if (!s_pos_hold.stick_in_deadzone_prev) {
            s_pos_hold.target_n = current_n;
            s_pos_hold.target_e = current_e;
            PID_Reset(&s_pos_hold.pid_pos_n);
            PID_Reset(&s_pos_hold.pid_pos_e);
            s_pos_hold.stick_in_deadzone_prev = true;
        }

        target_vn = PID_Compute_To_Error(&s_pos_hold.pid_pos_n,
                                         s_pos_hold.target_n - current_n,
                                         dt);
        target_ve = PID_Compute_To_Error(&s_pos_hold.pid_pos_e,
                                         s_pos_hold.target_e - current_e,
                                         dt);
    }

    cmd_n = PID_Compute_To_Measure(&s_pos_hold.pid_vel_n,
                                   target_vn - s_pos_hold.vel_n,
                                   s_pos_hold.vel_n,
                                   dt);
    cmd_e = PID_Compute_To_Measure(&s_pos_hold.pid_vel_e,
                                   target_ve - s_pos_hold.vel_e,
                                   s_pos_hold.vel_e,
                                   dt);

    body_tilt_from_ne(cmd_n, cmd_e, yaw_deg,
                      &s_pos_hold.roll_angle,
                      &s_pos_hold.pitch_angle);
    pos_hold_clamp_tilt(&s_pos_hold.roll_angle, &s_pos_hold.pitch_angle);
}

float PID_PosHold_GetRollAngle(void)
{
    return s_pos_hold.roll_angle;
}

float PID_PosHold_GetPitchAngle(void)
{
    return s_pos_hold.pitch_angle;
}
