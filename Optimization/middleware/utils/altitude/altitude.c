#include "utils/altitude/altitude.h"
#include "utils/madgwick/Madgwick_Fusion.h"
#include "utils/pid/pid.h"
#include "stdbool.h"

#define CRSF_CHANNEL_MIN        172U
#define CRSF_CHANNEL_MAX        1811U
#define CRSF_CHANNEL_MID        992U
#define CRSF_CHANNEL_SPAN       (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)

#define KP_RATE_ALT             0.15f
#define KI_RATE_ALT             0.05f
#define KD_RATE_ALT             0.005f

#define MAX_OUT_RATE_ALT        0.25f
#define MAX_OUT_I_RATE_ALT      0.15f

#define KP_LOCA_ALT             1.2f
#define KI_LOCA_ALT             0.0f
#define KD_LOCA_ALT             0.0f

#define MAX_OUT_LOCA_ALT        1.2f
#define MAX_OUT_I_LOCA_ALT      0.0f

#define STICK_DEADZONE          0.05f
#define MAX_CLIMB_RATE          1.2f
#define MAX_DESCEND_RATE        0.5f

#define DEFAULT_HOVER_THROTTLE  0.63f
#define HOVER_THROTTLE_MIN      0.2f
#define HOVER_THROTTLE_MAX      0.62f
#define OUTPUT_THROTTLE_MIN     0.10f
#define OUTPUT_THROTTLE_MAX     0.85f

typedef struct {
    PID_Controller pid_alt_z;
    PID_Controller pid_alt_vz;
    float target_height;
    float base_hover_throttle;
    bool stick_in_deadzone_prev;
} AltHold_State_t;

static AltHold_State_t s_alt_hold;
static volatile float s_throttle_output = DEFAULT_HOVER_THROTTLE;

static void alt_hold_set_throttle_output(float throttle)
{
    if (throttle > OUTPUT_THROTTLE_MAX) {
        throttle = OUTPUT_THROTTLE_MAX;
    } else if (throttle < OUTPUT_THROTTLE_MIN) {
        throttle = OUTPUT_THROTTLE_MIN;
    }

    s_throttle_output = throttle;
}

static void alt_hold_clamp_hover_throttle(void)
{
    if (s_alt_hold.base_hover_throttle > HOVER_THROTTLE_MAX) {
        s_alt_hold.base_hover_throttle = HOVER_THROTTLE_MAX;
    } else if (s_alt_hold.base_hover_throttle < HOVER_THROTTLE_MIN) {
        s_alt_hold.base_hover_throttle = HOVER_THROTTLE_MIN;
    }
}

void PID_AltHold_Init(void)
{
    s_alt_hold.target_height = 0.0f;
    s_alt_hold.base_hover_throttle = DEFAULT_HOVER_THROTTLE;
    s_alt_hold.stick_in_deadzone_prev = true;

    PID_Init(&s_alt_hold.pid_alt_z, KP_LOCA_ALT, KI_LOCA_ALT, KD_LOCA_ALT,
             MAX_OUT_LOCA_ALT, MAX_OUT_I_LOCA_ALT);
    PID_Init(&s_alt_hold.pid_alt_vz, KP_RATE_ALT, KI_RATE_ALT, KD_RATE_ALT,
             MAX_OUT_RATE_ALT, MAX_OUT_I_RATE_ALT);
    alt_hold_set_throttle_output(DEFAULT_HOVER_THROTTLE);
}

void PID_AltHold_Reset(void)
{
    s_alt_hold.target_height = 0.0f;
    s_alt_hold.base_hover_throttle = DEFAULT_HOVER_THROTTLE;
    s_alt_hold.stick_in_deadzone_prev = true;

    PID_Reset(&s_alt_hold.pid_alt_z);
    PID_Reset(&s_alt_hold.pid_alt_vz);
    alt_hold_set_throttle_output(DEFAULT_HOVER_THROTTLE);
}

void PID_AltHold_Enable(float current_manual_throttle)
{
    const MadgwickFusion_State_t *ahrs = MadgwickFusion_GetStatePtr();

    s_alt_hold.target_height = ahrs->altitude_state.altitude;
    s_alt_hold.base_hover_throttle = DEFAULT_HOVER_THROTTLE;
    alt_hold_clamp_hover_throttle();

    PID_Reset(&s_alt_hold.pid_alt_z);
    PID_Reset(&s_alt_hold.pid_alt_vz);
    s_alt_hold.stick_in_deadzone_prev = true;
    alt_hold_set_throttle_output(s_alt_hold.base_hover_throttle);
}

float PID_AltHold_StickNorm(uint16_t raw)
{
    float norm = (float)((int32_t)raw - (int32_t)CRSF_CHANNEL_MID) / (float)CRSF_CHANNEL_SPAN;

    if (norm > 1.0f) {
        norm = 1.0f;
    } else if (norm < -1.0f) {
        norm = -1.0f;
    }

    return norm;
}

float PID_AltHold_Update(float rc_stick_norm, float dt)
{
    const MadgwickFusion_State_t *ahrs = MadgwickFusion_GetStatePtr();

    float current_z = ahrs->altitude_state.altitude;
    float current_vz = ahrs->altitude_state.vertical_velocity;

    float target_vz = 0.0f;
    float height_error = 0.0f;

    bool stick_in_deadzone = (rc_stick_norm >= -STICK_DEADZONE && rc_stick_norm <= STICK_DEADZONE);

    if (!stick_in_deadzone) {
        target_vz = (rc_stick_norm > 0.0f)
            ? (rc_stick_norm * MAX_CLIMB_RATE)
            : (rc_stick_norm * MAX_DESCEND_RATE);

        s_alt_hold.target_height += target_vz * dt;
        s_alt_hold.stick_in_deadzone_prev = false;
        height_error = s_alt_hold.target_height - current_z;
    } else {
        if (!s_alt_hold.stick_in_deadzone_prev) {
            s_alt_hold.target_height = current_z;
            PID_Reset(&s_alt_hold.pid_alt_z);
            s_alt_hold.stick_in_deadzone_prev = true;
        }

        height_error = s_alt_hold.target_height - current_z;
        target_vz = PID_Compute_To_Error(&s_alt_hold.pid_alt_z, height_error, dt);
    }

    float vz_error = target_vz - current_vz;
    float delta_throttle = PID_Compute_To_Measure(&s_alt_hold.pid_alt_vz, vz_error, current_vz, dt);

    float final_throttle = s_alt_hold.base_hover_throttle + delta_throttle;
    alt_hold_set_throttle_output(final_throttle);
    return s_throttle_output;
}

float PID_AltHold_GetThrottle(void)
{
    return s_throttle_output;
}
