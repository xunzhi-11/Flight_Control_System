/**
 * @file    madgwick_fusion.c
 * @brief   Madgwick AHRS 姿态融合算法实现
 *
 * 坐标系：地球 NED + 机体 FRD，传感器数据不经轴翻转直接参与融合。
 */

#include "madgwick_fusion.h"
#include <math.h>
#include <string.h>
#include "app_core/app_sensor_data_center.h"
#include "data_structure_types.h"

/*============================================================================*/
/*                              私有宏定义                                     */
/*============================================================================*/
#ifndef clampf
#define clampf(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))
#endif


#define DEG_TO_RAD      (0.017453292519943295f)  /* π/180 */
#define RAD_TO_DEG      (57.29577951308232f)     /* 180/π */

#define DEFAULT_SEA_LEVEL_PRESSURE  101325.0f   /* 标准海平面气压 (Pa) */
#define GRAVITY_EARTH               9.80665f    /* 重力加速度 (m/s²) */

/* 气压高度公式常数 */
#define BARO_ALTITUDE_COEFF         44330.0f
#define BARO_ALTITUDE_EXP           0.1903f

/* 默认阈值 */
#define ACC_THRESHOLD_LOW           0.5f        /* 加速度下限 (g) */
#define ACC_THRESHOLD_HIGH          3.0f        /* 加速度上限 (g) */
#define ALTITUDE_FILTER_ALPHA       0.02f       /* 气压高度低通滤波系数 */
#define MAG_THRESHOLD_LOW           0.0f       /* uT, 根据实际环境调整 */
#define MAG_THRESHOLD_HIGH          400.0f      /* uT */
#define BARO_ALT_CORRECT_GAIN       0.25f      /* 气压高度修正增益（相对高度以 baro 为主） */
#define BARO_VEL_CORRECT_GAIN       0.10f      /* 气压速度修正增益 */
#define ALTITUDE_VZ_CLAMP           5.0f        /* 垂直速度限幅 (m/s) */
#define BARO_ERR_CLAMP_M            1.0f        /* 单次修正允许的最大高度误差 (m) */
#define BARO_ERR_RESYNC_M           2.0f        /* 超过此误差时硬同步到气压高度 */
#define BARO_WARMUP_FILTER_ALPHA    0.15f       /* 标定前预热滤波（比运行时更快） */

/* 垂直速度估计辅助 */
#define STATIONARY_GYRO_SUM_THRESHOLD_DEG   15.0f   /* |gx|+|gy|+|gz| 低于此视为近似静止 */
#define ACC_UP_BIAS_LEARN_ALPHA             0.0015f /* 静止时天向加速度偏置学习率 */
#define BARO_VZ_LPF_ALPHA                   0.20f   /* 气压差分速度低通 */
#define STATIONARY_VZ_BLEND                 0.65f   /* 静止时融合速度中气压差分权重 */
#define MOVING_VZ_BLEND                     0.20f   /* 运动时融合速度中气压差分权重 */
#define STATIONARY_VZ_ZERO_THRESHOLD        0.04f   /* 静止 ZUPT 门限 (m/s) */

/* 动态初始化相关配置 */
#define INIT_SAMPLE_COUNT           500U       /* 需要的静止样本数（约 0.5s @1kHz） */
#define INIT_GYRO_THRESHOLD_DEG     10.0f       /* 静止判定角速度阈值 (°/s) */


/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/*磁力计插值处理*/
typedef struct {
    float mag_x, mag_y, mag_z;
    uint32_t timestamp;
    uint8_t valid;
} MagSample_t;

/* 双缓冲存储最近两次磁力计数据 */
static MagSample_t s_mag_buffer[2] = {0};
static uint8_t s_mag_buffer_idx = 0;

/* 高度相关状态变量 */
static uint8_t s_altitude_initialized = 0;
static float s_last_raw_altitude = 0.0f;
static float s_ins_altitude = 0.0f;     /* IMU 积分高度 (m, 向上为正) */
static float s_ins_vz = 0.0f;           /* IMU 积分垂直速度 (m/s, 向上为正) */
static float s_acc_up_bias_est = 0.0f;    /* 静止时学习的天向加速度偏置 (m/s²) */
static float s_baro_vz_lpf = 0.0f;        /* 气压差分垂直速度低通 (m/s) */
static float s_prev_baro_alt = 0.0f;    /* 上一帧相对气压高度 (m) */
static uint8_t s_prev_baro_alt_valid = 0;

/* 融合状态 */
static MadgwickFusion_State_t s_state;

/* 配置参数 */
static MadgwickFusion_Config_t s_config = {
    .beta                   = MADGWICK_DEFAULT_BETA,
    .zeta                   = MADGWICK_DEFAULT_ZETA,
    .mag_declination        = 0.0f,
    .acc_threshold_low      = ACC_THRESHOLD_LOW,
    .acc_threshold_high     = ACC_THRESHOLD_HIGH,
    .altitude_filter_alpha  = ALTITUDE_FILTER_ALPHA,
    .mag_threshold_low      = MAG_THRESHOLD_LOW,
    .mag_threshold_high     = MAG_THRESHOLD_HIGH,
    .baro_alt_correct_gain  = BARO_ALT_CORRECT_GAIN,
    .baro_vel_correct_gain  = BARO_VEL_CORRECT_GAIN,
};

/* 磁力计上次时间戳 */
static uint32_t s_last_mag_time = 0;

/* 气压计上次时间戳 */
static uint32_t s_last_baro_time = 0;

/* imu上次时间戳 */
static uint32_t s_last_imu_time = 0;

/* 本周期 imu_update() 结果，供控制环只读复用，避免重复采样 */
static const IMU_Scaled_t *s_imu_snap = NULL;

/* 高度参考基准 */
static float s_altitude_offset = 0.0f;

/* 姿态动态初始化累积 */
static float s_init_acc_sum[3] = {0.0f, 0.0f, 0.0f};
static float s_init_mag_sum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t s_init_sample_count = 0U;

/*============================================================================*/
/*                              私有函数声明                                   */
/*============================================================================*/

static float invSqrt(float x);
static void normalizeQuaternion(void);
static void computeEulerAngles(void);
static void computeLinearAcceleration(float ax, float ay, float az);
static float computeBaroAltitude(float pressure, float temperature);
static float computeVerticalAccUp(void);
static void altitudeImuStep(float dt);
static void altitudeResetAuxState(void);
static bool altitudeFusionEnabled(void);
static bool altitudeIsStationary(void);
static void fusionSetQuaternionFromEuler(float roll, float pitch, float yaw);
static void fusionResetDynamicInit(void);
static void fusionDynamicInitStep(float gx, float gy, float gz,
                                  float ax, float ay, float az,
                                  float mx, float my, float mz);

/*============================================================================*/
/*                              快速平方根倒数                                  */
/*============================================================================*/

/**
 * @brief  快速计算 1/sqrt(x) - 使用牛顿迭代法
 */
#include <string.h>   // 需要包含该头文件

static float invSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    
    float halfx = 0.5f * x;
    float y = x;
    long i;
    memcpy(&i, &y, sizeof(i));      // 将 y 的位模式复制到 i
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));      // 将 i 的位模式复制回 y
    
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*============================================================================*/
/*                              四元数归一化                                   */
/*============================================================================*/

static void normalizeQuaternion(void)
{
    float recipNorm = invSqrt(s_state.quaternion.w * s_state.quaternion.w +
                              s_state.quaternion.x * s_state.quaternion.x +
                              s_state.quaternion.y * s_state.quaternion.y +
                              s_state.quaternion.z * s_state.quaternion.z);
    
    s_state.quaternion.w *= recipNorm;
    s_state.quaternion.x *= recipNorm;
    s_state.quaternion.y *= recipNorm;
    s_state.quaternion.z *= recipNorm;
}

/*============================================================================*/
/*                              欧拉角计算                                     */
/*============================================================================*/

static void computeEulerAngles(void)
{
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;
    
    /* Roll (X轴旋转) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    s_state.euler.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    /* Pitch (Y轴旋转) - 处理万向锁 */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        s_state.euler.pitch = copysignf(90.0f, sinp);
    } else {
        s_state.euler.pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    /* Yaw (Z轴旋转) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    s_state.euler.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    
    /* 添加磁偏角校正 */
    //s_state.euler.yaw += s_config.mag_declination;
    
    /* 归一化 yaw 到 -180 ~ 180 */
    if (s_state.euler.yaw > 180.0f) {
        s_state.euler.yaw -= 360.0f;
    } else if (s_state.euler.yaw < -180.0f) {
        s_state.euler.yaw += 360.0f;
    }
}

/*============================================================================*/
/*                              线性加速度计算 (去除重力)                        */
/*============================================================================*/

static void computeLinearAcceleration(float ax, float ay, float az)
{
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;
    
    /* 重力在机体坐标系中的分量 */
    float gx = 2.0f * (q1 * q3 - q0 * q2);
    float gy = 2.0f * (q0 * q1 + q2 * q3);
    float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    /* 线性加速度 = 测量加速度 - 重力 */
    s_state.linear_acc_x = ax - gx;
    s_state.linear_acc_y = ay - gy;
    s_state.linear_acc_z = az - gz;
}

/*============================================================================*/
/*                              气压高度计算                                   */
/*============================================================================*/

static float computeBaroAltitude(float pressure, float temperature)
{
    if (pressure <= 0.0f) {
        return 0.0f;
    }
    
    float seaLevel = s_state.altitude_state.sea_level_pressure;
    
    /* 国际气压高度公式 */
    float altitude = BARO_ALTITUDE_COEFF * 
                     (1.0f - powf(pressure / seaLevel, BARO_ALTITUDE_EXP));
    
    return altitude;
}

/**
 * @brief  计算机体去重力加速度在 NED 天向的分量 (向上为正, m/s²)
 */
static float computeVerticalAccUp(void)
{
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;

    /* NED 地向单位向量在机体 FRD 中的分量 */
    float down_x = 2.0f * (q1 * q3 - q0 * q2);
    float down_y = 2.0f * (q0 * q1 + q2 * q3);
    float down_z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float acc_up_g = -(s_state.linear_acc_x * down_x
                     + s_state.linear_acc_y * down_y
                     + s_state.linear_acc_z * down_z);

    return acc_up_g * GRAVITY_EARTH;
}

static void altitudeResetAuxState(void)
{
    s_acc_up_bias_est = 0.0f;
    s_baro_vz_lpf = 0.0f;
    s_prev_baro_alt = 0.0f;
    s_prev_baro_alt_valid = 0U;
}

static bool altitudeFusionEnabled(void)
{
    return BaroCalibrator_IsReady(baro_calib_get_instance());
}

static bool altitudeIsStationary(void)
{
    if (s_imu_snap == NULL) {
        return false;
    }

    const float gyro_sum = fabsf(s_imu_snap->gx) + fabsf(s_imu_snap->gy) + fabsf(s_imu_snap->gz);
    return gyro_sum < STATIONARY_GYRO_SUM_THRESHOLD_DEG;
}

/**
 * @brief  IMU 垂直加速度积分预测 (每 AHRS/IMU 步调用)
 */
static void altitudeImuStep(float dt)
{
    if (!altitudeFusionEnabled() || !s_altitude_initialized || dt <= 0.0f) {
        return;
    }

    float acc_up = computeVerticalAccUp();

    /* 静止时在线估计并扣除姿态-比力失配带来的天向加速度偏置 */
    if (altitudeIsStationary()) {
        s_acc_up_bias_est += ACC_UP_BIAS_LEARN_ALPHA * (acc_up - s_acc_up_bias_est);
        acc_up -= s_acc_up_bias_est;
    }

    s_ins_vz += acc_up * dt;
    s_ins_vz = clampf(s_ins_vz, -ALTITUDE_VZ_CLAMP, ALTITUDE_VZ_CLAMP);
    s_ins_altitude += s_ins_vz * dt;

    s_state.altitude_state.altitude = s_ins_altitude;
    s_state.altitude_state.vertical_velocity = s_ins_vz;
}

/*============================================================================*/
/*                      动态初始化：由静止 IMU+磁力计估算初始姿态                */
/*============================================================================*/

static void fusionSetQuaternionFromEuler(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    s_state.quaternion.w = cr * cp * cy + sr * sp * sy;
    s_state.quaternion.x = sr * cp * cy - cr * sp * sy;
    s_state.quaternion.y = cr * sp * cy + sr * cp * sy;
    s_state.quaternion.z = cr * cp * sy - sr * sp * cy;

    normalizeQuaternion();
    computeEulerAngles();
}

static void fusionResetDynamicInit(void)
{
    s_init_acc_sum[0] = s_init_acc_sum[1] = s_init_acc_sum[2] = 0.0f;
    s_init_mag_sum[0] = s_init_mag_sum[1] = s_init_mag_sum[2] = 0.0f;
    s_init_sample_count = 0U;
    s_state.is_initialized = 0U;
    s_state.gyro_bias.x = 0.0f;
    s_state.gyro_bias.y = 0.0f;
    s_state.gyro_bias.z = 0.0f;
}

static void fusionDynamicInitStep(float gx, float gy, float gz,
                                  float ax, float ay, float az,
                                  float mx, float my, float mz)
{
    if (s_state.is_initialized) {
        return;
    }

    /* 静止检测：角速度很小 + 加速度/磁力计在有效范围内 */
    float gyro_abs_sum = fabsf(gx) + fabsf(gy) + fabsf(gz); /* 单位：°/s */
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    float magNorm = sqrtf(mx * mx + my * my + mz * mz);

    uint8_t accValid = (accNorm > s_config.acc_threshold_low &&
                        accNorm < s_config.acc_threshold_high);
    uint8_t magValid = (magNorm > s_config.mag_threshold_low &&
                        magNorm < s_config.mag_threshold_high);

    if ((gyro_abs_sum < INIT_GYRO_THRESHOLD_DEG) && accValid && magValid) {
        s_init_acc_sum[0] += ax;
        s_init_acc_sum[1] += ay;
        s_init_acc_sum[2] += az;

        s_init_mag_sum[0] += mx;
        s_init_mag_sum[1] += my;
        s_init_mag_sum[2] += mz;

        if (s_init_sample_count < 0xFFFFFFFEU) {
            s_init_sample_count++;
        }
    } else {
        /* 若中途发生明显运动，则温和衰减已有样本，避免“坏姿态”被锁定 */
        if (s_init_sample_count > 0U) {
            s_init_sample_count /= 2U;
        }
    }

    if (s_init_sample_count >= INIT_SAMPLE_COUNT) {
        /* 计算平均加速度和磁场 */
        float acc_x = s_init_acc_sum[0] / (float)s_init_sample_count;
        float acc_y = s_init_acc_sum[1] / (float)s_init_sample_count;
        float acc_z = s_init_acc_sum[2] / (float)s_init_sample_count;

        float mag_x = s_init_mag_sum[0] / (float)s_init_sample_count;
        float mag_y = s_init_mag_sum[1] / (float)s_init_sample_count;
        float mag_z = s_init_mag_sum[2] / (float)s_init_sample_count;

        /* FRD/NED：水平静止时 acc ≈ (0,0,+1g) → roll=0, pitch=0 */
        float roll  = atan2f(acc_y, acc_z);
        float pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));

        /* 磁力计倾斜补偿后求航向角 */
        float sinRoll  = sinf(roll);
        float cosRoll  = cosf(roll);
        float sinPitch = sinf(pitch);
        float cosPitch = cosf(pitch);

        float mx2 = mag_x * cosPitch + mag_z * sinPitch;
        float my2 = mag_x * sinRoll * sinPitch + mag_y * cosRoll - mag_z * sinRoll * cosPitch;

        float yaw = atan2f(-my2, mx2);

        /* 加上磁偏角（转换为弧度） */
        yaw += s_config.mag_declination * DEG_TO_RAD;

        fusionSetQuaternionFromEuler(roll, pitch, yaw);

        s_state.is_initialized = 1U;
        s_init_sample_count = 0U;
    }
}

/*============================================================================*/
/*                              API 实现                                       */
/*============================================================================*/

void MadgwickFusion_Init(const MadgwickFusion_Config_t *config)
{
    /* 清空状态 */
    memset(&s_state, 0, sizeof(s_state));
    
    /* 初始化四元数为单位四元数（动态初始化完成前的占位） */
    s_state.quaternion.w = 1.0f;
    s_state.quaternion.x = 0.0f;
    s_state.quaternion.y = 0.0f;
    s_state.quaternion.z = 0.0f;
    
    /* 初始化高度状态 */
    s_state.altitude_state.sea_level_pressure = DEFAULT_SEA_LEVEL_PRESSURE;
    
    /* 应用配置 */
    if (config != NULL) {
        memcpy(&s_config, config, sizeof(s_config));
    }
    
    /* 重置内部变量 */
    s_last_mag_time = 0;
    s_last_baro_time = 0;
    s_altitude_offset = 0.0f;
    s_ins_altitude = 0.0f;
    s_ins_vz = 0.0f;
    s_last_raw_altitude = 0.0f;

    s_altitude_initialized = 0;
    altitudeResetAuxState();

    fusionResetDynamicInit();
}

void MadgwickFusion_Reset(void)
{
    MadgwickFusion_Init(NULL);
}

/*============================================================================*/
/*                              IMU 6轴更新 (无磁力计)                          */
/*============================================================================*/

void MadgwickFusion_UpdateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;
    
    /* 角速度转换为弧度/秒 */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
    
    gx -= s_state.gyro_bias.x;
    gy -= s_state.gyro_bias.y;
    gz -= s_state.gyro_bias.z;

    /* 四元数微分方程 (陀螺仪积分) */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    
    /* 计算加速度计模长 */
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    
    /* 仅在加速度有效范围内进行校正 */
    if (accNorm > s_config.acc_threshold_low && 
        accNorm < s_config.acc_threshold_high) {
        
        /* 归一化加速度计测量值 */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* 辅助变量 */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        /* 梯度下降算法校正步骤 */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay 
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        /* 归一化步长 */
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        /* 陀螺仪偏差估计 (使用 zeta 参数) */
        if (s_config.zeta > 0.0f) {
            float gyroErr_x = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
            float gyroErr_y = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
            float gyroErr_z = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);
            
            s_state.gyro_bias.x += s_config.zeta * dt * gyroErr_x;
            s_state.gyro_bias.y += s_config.zeta * dt * gyroErr_y;
            s_state.gyro_bias.z += s_config.zeta * dt * gyroErr_z;
            
            /* 限制偏差范围 (±0.5 rad/s ≈ ±28.6 °/s) */
            s_state.gyro_bias.x = clampf(s_state.gyro_bias.x, -0.5f, 0.5f);
            s_state.gyro_bias.y = clampf(s_state.gyro_bias.y, -0.5f, 0.5f);
            s_state.gyro_bias.z = clampf(s_state.gyro_bias.z, -0.5f, 0.5f);
        }
        
        /* 应用反馈步骤 */
        qDot1 -= s_config.beta * s0;
        qDot2 -= s_config.beta * s1;
        qDot3 -= s_config.beta * s2;
        qDot4 -= s_config.beta * s3;
    }
    
    /* 积分得到四元数 */
    s_state.quaternion.w += qDot1 * dt;
    s_state.quaternion.x += qDot2 * dt;
    s_state.quaternion.y += qDot3 * dt;
    s_state.quaternion.z += qDot4 * dt;
    
    /* 归一化四元数 */
    normalizeQuaternion();
    
    /* 计算欧拉角 */
    computeEulerAngles();
    
    /* 计算线性加速度 (使用原始加速度值) */
    computeLinearAcceleration(ax * accNorm, ay * accNorm, az * accNorm);
    altitudeImuStep(dt);
    
    /* 更新时间信息 */
    s_state.dt = dt;
    s_state.update_count++;
}

/*============================================================================*/
/*                              AHRS 9轴更新 (含磁力计)                         */
/*============================================================================*/

void MadgwickFusion_UpdateAHRS(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz,
                                float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3, q3q3;
    
    /* 参数检查 */
    if (dt <= 0.0f || dt > 1.0f) {
        return;
    }
    
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;
    
    /* 角速度转换为弧度/秒 */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
    
    /* 补偿陀螺仪偏差 */
    gx -= s_state.gyro_bias.x;
    gy -= s_state.gyro_bias.y;
    gz -= s_state.gyro_bias.z;
    
    /* 四元数微分方程 (陀螺仪积分) */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    
    /* 计算加速度计和磁力计模长 */
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    float magNorm = sqrtf(mx * mx + my * my + mz * mz);
    
    /* 检查数据有效性 */
    uint8_t accValid = (accNorm > s_config.acc_threshold_low && 
                        accNorm < s_config.acc_threshold_high);
    uint8_t magValid = (magNorm > s_config.mag_threshold_low && 
                        magNorm < s_config.mag_threshold_high);
    
    s_state.mag_valid = magValid;
    
    if (accValid && magValid) {
        /*====================================================================*/
        /*                         9轴融合                                    */
        /*====================================================================*/
        
        /* 归一化加速度计 */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* 归一化磁力计 */
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        /* 辅助变量 */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        
        /* 磁力计辅助变量 */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        
        /* 参考方向的地球磁场 (水平分量和垂直分量) */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 
           + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 
           - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 
             - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        
        /* 梯度下降校正步骤 (9轴完整公式) */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) 
           + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) 
           - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
           
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) 
           + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) 
           - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
           + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
           
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) 
           + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) 
           - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
           + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
           
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) 
           + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) 
           + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
           + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
           + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        /* 归一化步长 */
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        /* 陀螺仪偏差估计 */
        if (s_config.zeta > 0.0f) {
            float gyroErr_x = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
            float gyroErr_y = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
            float gyroErr_z = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);
            
            s_state.gyro_bias.x += s_config.zeta * dt * gyroErr_x;
            s_state.gyro_bias.y += s_config.zeta * dt * gyroErr_y;
            s_state.gyro_bias.z += s_config.zeta * dt * gyroErr_z;
            
            /* 限制偏差范围 */
            s_state.gyro_bias.x = clampf(s_state.gyro_bias.x, -0.5f, 0.5f);
            s_state.gyro_bias.y = clampf(s_state.gyro_bias.y, -0.5f, 0.5f);
            s_state.gyro_bias.z = clampf(s_state.gyro_bias.z, -0.5f, 0.5f);
        }
        
        /* 应用反馈步骤 */
        qDot1 -= s_config.beta * s0;
        qDot2 -= s_config.beta * s1;
        qDot3 -= s_config.beta * s2;
        qDot4 -= s_config.beta * s3;
        
    } else if (accValid) {
        /*====================================================================*/
        /*                    仅6轴融合 (磁力计无效时)                          */
        /*====================================================================*/
        
        /* 归一化加速度计 */
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* 辅助变量 */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        /* 6轴梯度下降 */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay 
           - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay 
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        /* 归一化步长 */
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        /* 陀螺仪偏差估计 */
        if (s_config.zeta > 0.0f) {
            float gyroErr_x = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
            float gyroErr_y = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
            float gyroErr_z = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);
            
            s_state.gyro_bias.x += s_config.zeta * dt * gyroErr_x;
            s_state.gyro_bias.y += s_config.zeta * dt * gyroErr_y;
            s_state.gyro_bias.z += s_config.zeta * dt * gyroErr_z;
            
            s_state.gyro_bias.x = clampf(s_state.gyro_bias.x, -0.5f, 0.5f);
            s_state.gyro_bias.y = clampf(s_state.gyro_bias.y, -0.5f, 0.5f);
            s_state.gyro_bias.z = clampf(s_state.gyro_bias.z, -0.5f, 0.5f);
        }
        
        /* 应用反馈步骤 */
        qDot1 -= s_config.beta * s0;
        qDot2 -= s_config.beta * s1;
        qDot3 -= s_config.beta * s2;
        qDot4 -= s_config.beta * s3;
    }
    /* 如果加速度也无效，则仅使用陀螺仪积分 */
    
    /* 积分得到四元数 */
    s_state.quaternion.w += qDot1 * dt;
    s_state.quaternion.x += qDot2 * dt;
    s_state.quaternion.y += qDot3 * dt;
    s_state.quaternion.z += qDot4 * dt;
    
    /* 归一化四元数 */
    normalizeQuaternion();
    
    /* 计算欧拉角 */
    computeEulerAngles();
    
    /* 计算线性加速度 */
    computeLinearAcceleration(ax * accNorm, ay * accNorm, az * accNorm);
    altitudeImuStep(dt);
    
    /* 更新时间信息 */
    s_state.dt = dt;
    s_state.update_count++;
}

/*============================================================================*/
/*                              高度更新                                       */
/*============================================================================*/

void MadgwickFusion_UpdateAltitude(float pressure, float temperature, float dt)
{
    if (pressure <= 0.0f || dt <= 0.0f) {
        s_state.baro_valid = 0;
        return;
    }

    if (!altitudeFusionEnabled()) {
        /* 标定完成前较快预热气压高度，避免零点用到未收敛的滤波值 */
        float raw_altitude = computeBaroAltitude(pressure, temperature);
        if (raw_altitude > 0.0f) {
            const float alpha = BARO_WARMUP_FILTER_ALPHA;
            s_last_raw_altitude = s_last_raw_altitude * (1.0f - alpha) + raw_altitude * alpha;
        }
        s_state.baro_valid = 0;
        return;
    }
    
    s_state.baro_valid = 1;
    s_state.altitude_state.temperature = temperature;
    
    float raw_altitude = computeBaroAltitude(pressure, temperature);
    
    if (!s_altitude_initialized) {
        if (s_last_raw_altitude <= 0.0f) {
            s_last_raw_altitude = raw_altitude;
        }
        s_ins_altitude = 0.0f;
        s_ins_vz = 0.0f;
        s_state.altitude_state.altitude = 0.0f;
        s_state.altitude_state.vertical_velocity = 0.0f;
        s_altitude_initialized = 1;
        altitudeResetAuxState();
        return;
    }
    
    /* 气压高度轻滤波，抑制测量噪声后再做修正 */
    float alpha = s_config.altitude_filter_alpha;
    float filtered_baro = s_last_raw_altitude * (1.0f - alpha) + raw_altitude * alpha;
    s_last_raw_altitude = filtered_baro;

    float baro_alt = filtered_baro - s_altitude_offset;
    float err = baro_alt - s_ins_altitude;

    if (s_prev_baro_alt_valid) {
        float baro_vz_inst = (baro_alt - s_prev_baro_alt) / dt;
        baro_vz_inst = clampf(baro_vz_inst, -ALTITUDE_VZ_CLAMP, ALTITUDE_VZ_CLAMP);
        s_baro_vz_lpf += BARO_VZ_LPF_ALPHA * (baro_vz_inst - s_baro_vz_lpf);
    } else {
        s_baro_vz_lpf = 0.0f;
    }
    s_prev_baro_alt = baro_alt;
    s_prev_baro_alt_valid = 1U;

    if (fabsf(err) > BARO_ERR_RESYNC_M) {
        s_ins_altitude = baro_alt;
        s_ins_vz = s_baro_vz_lpf;
        err = 0.0f;
    } else if (altitudeIsStationary()) {
        /* 静止：相对高度直接跟随气压，避免 IMU 积分漂移 */
        s_ins_altitude = baro_alt;
        s_ins_vz = s_baro_vz_lpf;
        if (fabsf(s_ins_vz) < STATIONARY_VZ_ZERO_THRESHOLD) {
            s_ins_vz = 0.0f;
        }
    } else {
        err = clampf(err, -BARO_ERR_CLAMP_M, BARO_ERR_CLAMP_M);
        s_ins_altitude += err * s_config.baro_alt_correct_gain;
        s_ins_vz += err * s_config.baro_vel_correct_gain;

        {
            const float vz_blend = MOVING_VZ_BLEND;
            s_ins_vz = s_ins_vz * (1.0f - vz_blend) + s_baro_vz_lpf * vz_blend;
        }
    }

    s_ins_vz = clampf(s_ins_vz, -ALTITUDE_VZ_CLAMP, ALTITUDE_VZ_CLAMP);

    s_state.altitude_state.altitude = s_ins_altitude;
    s_state.altitude_state.vertical_velocity = s_ins_vz;
}

/*============================================================================*/
/*                              获取状态                                       */
/*============================================================================*/

const MadgwickFusion_State_t* MadgwickFusion_GetStatePtr(void)
{
    return &s_state;
}

const IMU_Scaled_t* MadgwickFusion_GetImuSnap(void)
{
    return s_imu_snap;
}

void MadgwickFusion_GetQuaternion(float q[4])
{
    if (q != NULL) {
        q[0] = s_state.quaternion.w;
        q[1] = s_state.quaternion.x;
        q[2] = s_state.quaternion.y;
        q[3] = s_state.quaternion.z;
    }
}

void MadgwickFusion_GetEuler(float euler[3])
{
    if (euler != NULL) {
        euler[0] = s_state.euler.roll;
        euler[1] = s_state.euler.pitch;
        euler[2] = s_state.euler.yaw;
    }
}

void MadgwickFusion_GetLinearAcceleration(float acc[3])
{
    if (acc != NULL) {
        acc[0] = s_state.linear_acc_x;
        acc[1] = s_state.linear_acc_y;
        acc[2] = s_state.linear_acc_z;
    }
}

/*============================================================================*/
/*                              参数设置                                       */
/*============================================================================*/

void MadgwickFusion_SetSeaLevelPressure(float pressure)
{
    if (pressure > 80000.0f && pressure < 120000.0f) {
        s_state.altitude_state.sea_level_pressure = pressure;
    }
}

void MadgwickFusion_CalibrateAltitudeAtPressure(float pressure_pa, float temperature_c)
{
    const float raw_alt = computeBaroAltitude(pressure_pa, temperature_c);

    s_last_raw_altitude = raw_alt;
    s_altitude_offset = raw_alt;
    s_ins_altitude = 0.0f;
    s_ins_vz = 0.0f;
    s_state.altitude_state.altitude = 0.0f;
    s_state.altitude_state.vertical_velocity = 0.0f;
    s_state.altitude_state.temperature = temperature_c;
    s_altitude_initialized = 1U;
    altitudeResetAuxState();
    s_prev_baro_alt = 0.0f;
    s_prev_baro_alt_valid = 1U;
}

void MadgwickFusion_CalibrateAltitude(void)
{
    s_altitude_offset = s_last_raw_altitude;
    s_ins_altitude = 0.0f;
    s_ins_vz = 0.0f;
    s_state.altitude_state.altitude = 0.0f;
    s_state.altitude_state.vertical_velocity = 0.0f;
    s_altitude_initialized = 1U;
    altitudeResetAuxState();
    s_prev_baro_alt = 0.0f;
    s_prev_baro_alt_valid = 1U;
}

void MadgwickFusion_SetBeta(float beta)
{
    s_config.beta = clampf(beta, 0.0f, 1.0f);
}

void MadgwickFusion_SetZeta(float zeta)
{
    s_config.zeta = clampf(zeta, 0.0f, 0.1f);
}

void MadgwickFusion_SetMagDeclination(float declination)
{
    s_config.mag_declination = clampf(declination, -180.0f, 180.0f);
}

void MadgwickFusion_ResetGyroBias(void)
{
    s_state.gyro_bias.x = 0.0f;
    s_state.gyro_bias.y = 0.0f;
    s_state.gyro_bias.z = 0.0f;
}

/*============================================================================*/
/*                              坐标变换                                       */
/*============================================================================*/

void MadgwickFusion_GetRotationMatrix(float matrix[9])
{
    if (matrix == NULL) return;
    
    float q0 = s_state.quaternion.w;
    float q1 = s_state.quaternion.x;
    float q2 = s_state.quaternion.y;
    float q3 = s_state.quaternion.z;
    
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    /* 旋转矩阵 (机体到地球) - 行优先存储 */
    matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  /* R11 */
    matrix[1] = 2.0f * (q1q2 - q0q3);        /* R12 */
    matrix[2] = 2.0f * (q1q3 + q0q2);        /* R13 */
    
    matrix[3] = 2.0f * (q1q2 + q0q3);        /* R21 */
    matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;  /* R22 */
    matrix[5] = 2.0f * (q2q3 - q0q1);        /* R23 */
    
    matrix[6] = 2.0f * (q1q3 - q0q2);        /* R31 */
    matrix[7] = 2.0f * (q2q3 + q0q1);        /* R32 */
    matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;  /* R33 */
}

void MadgwickFusion_BodyToEarth(float bx, float by, float bz,
                                 float *ex, float *ey, float *ez)
{
    if (ex == NULL || ey == NULL || ez == NULL) return;
    
    float matrix[9];
    MadgwickFusion_GetRotationMatrix(matrix);
    
    *ex = matrix[0] * bx + matrix[1] * by + matrix[2] * bz;
    *ey = matrix[3] * bx + matrix[4] * by + matrix[5] * bz;
    *ez = matrix[6] * bx + matrix[7] * by + matrix[8] * bz;
}

void MadgwickFusion_EarthToBody(float ex, float ey, float ez,
                                 float *bx, float *by, float *bz)
{
    if (bx == NULL || by == NULL || bz == NULL) return;
    
    float matrix[9];
    MadgwickFusion_GetRotationMatrix(matrix);
    
    /* 旋转矩阵的转置 (地球到机体) */
    *bx = matrix[0] * ex + matrix[3] * ey + matrix[6] * ez;
    *by = matrix[1] * ex + matrix[4] * ey + matrix[7] * ez;
    *bz = matrix[2] * ex + matrix[5] * ey + matrix[8] * ez;
}

/*============================================================================*/
/*                    主更新函数 - 基础版本（无插值）                            */
/*============================================================================*/
void MadgwickFusion_Update(void)
{
    const IMU_Scaled_t *imu_snap;
    MAGNETOMETER_Calib_t* mag_snap;
    BAROMETER_Real_t* baro_snap;
    
    imu_update();
    imu_snap = imu_get_fusion_imu();
    s_imu_snap = imu_get_gyro_rate();
    baro_snap = baro_update() ; 
    mag_snap = mag_update() ; 
    
    if (imu_snap->timestamp == s_last_imu_time) return;
    s_last_imu_time = imu_snap->timestamp;
    
    float dt_s = imu_snap->dt;
    /* 安全检查 */
    
    if (dt_s < 0.0002f || dt_s > 0.0005f) {
        dt_s = 0.00025f;
    }
    

    /* 磁力计：有有效数据则每步都参与 9 轴融合，避免 yaw 仅在 10Hz 时被校正导致漂移 */
    if (mag_snap->god_time != s_last_mag_time) {
        s_last_mag_time = mag_snap->god_time;
    }
    float magNorm = sqrtf(mag_snap->mag_x * mag_snap->mag_x + mag_snap->mag_y * mag_snap->mag_y + mag_snap->mag_z * mag_snap->mag_z);
    uint8_t mag_valid = (magNorm > s_config.mag_threshold_low && magNorm < s_config.mag_threshold_high)
                        && (s_last_mag_time != 0u); 

    /* 上电后动态初始化：静止阶段用 FRD 传感器数据估算初始四元数 */
    if (!s_state.is_initialized) {
        if (mag_valid) {
            fusionDynamicInitStep(
                imu_snap->gx, imu_snap->gy, imu_snap->gz,
                imu_snap->ax, imu_snap->ay, imu_snap->az,
                mag_snap->mag_x, mag_snap->mag_y, mag_snap->mag_z
            );
        }
        s_state.timestamp_us = imu_snap->timestamp;

        if (!s_state.is_initialized) {
            return;
        }
    }

    if (mag_valid) {
        MadgwickFusion_UpdateAHRS(
            imu_snap->gx, imu_snap->gy, imu_snap->gz,
            imu_snap->ax, imu_snap->ay, imu_snap->az,
            mag_snap->mag_x, mag_snap->mag_y, mag_snap->mag_z,
            dt_s
        );
    } else {
        MadgwickFusion_UpdateIMU(
            imu_snap->gx, imu_snap->gy, imu_snap->gz,
            imu_snap->ax, imu_snap->ay, imu_snap->az,
            dt_s
        );
    }
    
    /* 气压计独立更新 */
    if (baro_snap->god_time != s_last_baro_time) {
        float baro_dt_s = 0.005f;  // 默认5ms
        if (s_last_baro_time != 0) {
            baro_dt_s = baro_snap->dt;
            if (baro_dt_s < 0.002f || baro_dt_s > 0.05f) {
                baro_dt_s = 0.005f;
            }
        }
        s_last_baro_time = baro_snap->god_time;
        MadgwickFusion_UpdateAltitude((float)baro_snap->press, (float)baro_snap->temp, baro_dt_s);
    }
    
    s_state.timestamp_us = imu_snap->timestamp;
}

/*============================================================================*/
/*                    主更新函数 - 基础版本（无插值）(无磁力计)                            */
/*============================================================================*/
void MadgwickFusion_Update_Nomag(void)
{
    const IMU_Scaled_t *imu_snap;
    BAROMETER_Real_t* baro_snap;
    
    imu_update();
    imu_snap = imu_get_fusion_imu();
    s_imu_snap = imu_get_gyro_rate();
    baro_snap = baro_update() ; 
    
    if (imu_snap->timestamp == s_last_imu_time) return;
    s_last_imu_time = imu_snap->timestamp;
    
    float dt_s = imu_snap->dt;
    /* 安全检查 */
    
    if (dt_s < 0.0002f || dt_s > 0.0005f) {
        dt_s = 0.00025f;
    }

    /* 6 轴路径：静止时用加速度计估算 roll/pitch，yaw 置 0 */
    if (!s_state.is_initialized) {
        float gyro_abs_sum = fabsf(imu_snap->gx) + fabsf(imu_snap->gy) + fabsf(imu_snap->gz);
        float accNorm = sqrtf(imu_snap->ax * imu_snap->ax
                            + imu_snap->ay * imu_snap->ay
                            + imu_snap->az * imu_snap->az);
        uint8_t accValid = (accNorm > s_config.acc_threshold_low
                         && accNorm < s_config.acc_threshold_high);

        if ((gyro_abs_sum < INIT_GYRO_THRESHOLD_DEG) && accValid) {
            float roll  = atan2f(imu_snap->ay, imu_snap->az);
            float pitch = atan2f(-imu_snap->ax,
                                 sqrtf(imu_snap->ay * imu_snap->ay
                                      + imu_snap->az * imu_snap->az));
            fusionSetQuaternionFromEuler(roll, pitch, 0.0f);
            s_state.is_initialized = 1U;
        }

        s_state.timestamp_us = imu_snap->timestamp;
        if (!s_state.is_initialized) {
            return;
        }
    }

    MadgwickFusion_UpdateIMU(
        imu_snap->gx, imu_snap->gy, imu_snap->gz,
        imu_snap->ax, imu_snap->ay, imu_snap->az,
        dt_s
    );
    /* 气压计独立更新 */
    if (baro_snap->god_time != s_last_baro_time) {
        float baro_dt_s = 0.005f;  // 默认5ms
        if (s_last_baro_time != 0) {
            baro_dt_s = baro_snap->dt;
            if (baro_dt_s < 0.002f || baro_dt_s > 0.05f) {
                baro_dt_s = 0.005f;
            }
        }
        s_last_baro_time = baro_snap->god_time;
        MadgwickFusion_UpdateAltitude((float)baro_snap->press, (float)baro_snap->temp, baro_dt_s);
    }
    
    s_state.timestamp_us = imu_snap->timestamp;
}



/*============================================================================*/
/*                         磁力计数据插值结构                                   */
/*============================================================================*/

/**
 * @brief  更新磁力计缓冲区
 */
static void updateMagBuffer(const MAGNETOMETER_Calib_t* mag)
{
    s_mag_buffer_idx = (s_mag_buffer_idx + 1) % 2;
    s_mag_buffer[s_mag_buffer_idx].mag_x = mag->mag_x;
    s_mag_buffer[s_mag_buffer_idx].mag_y = mag->mag_y;
    s_mag_buffer[s_mag_buffer_idx].mag_z = mag->mag_z;
    s_mag_buffer[s_mag_buffer_idx].timestamp = mag->god_time;
    s_mag_buffer[s_mag_buffer_idx].valid = 1;
}

/**
 * @brief  线性插值获取指定时刻的磁力计数据
 * @param  target_time: 目标时间戳 (IMU 的 god_time)
 * @param  mx, my, mz: 输出插值后的磁力计数据
 * @retval 1=成功, 0=数据不足
 */
static uint8_t interpolateMag(uint32_t target_time, 
                               float* mx, float* my, float* mz)
{
    MagSample_t* newer = &s_mag_buffer[s_mag_buffer_idx];
    MagSample_t* older = &s_mag_buffer[(s_mag_buffer_idx + 1) % 2];
    
    /* 检查数据有效性 */
    if (!newer->valid || !older->valid) {
        /* 数据不足，使用最新值 */
        if (newer->valid) {
            *mx = newer->mag_x;
            *my = newer->mag_y;
            *mz = newer->mag_z;
            return 1;
        }
        return 0;
    }
    
    /* 计算插值系数 */
    uint32_t dt_total = newer->timestamp - older->timestamp;
    if (dt_total == 0) {
        *mx = newer->mag_x;
        *my = newer->mag_y;
        *mz = newer->mag_z;
        return 1;
    }
    
    /* 目标时间在两个采样点之间的位置 */
    float alpha;
    if (target_time >= newer->timestamp) {
        /* 外推（不推荐，但允许小范围） */
        alpha = 1.0f;
    } else if (target_time <= older->timestamp) {
        alpha = 0.0f;
    } else {
        alpha = (float)(target_time - older->timestamp) / (float)dt_total;
    }
    
    /* 线性插值 */
    *mx = older->mag_x + alpha * (newer->mag_x - older->mag_x);
    *my = older->mag_y + alpha * (newer->mag_y - older->mag_y);
    *mz = older->mag_z + alpha * (newer->mag_z - older->mag_z);
    
    return 1;
}


void MadgwickFusion_UpdateWithInterpolation(void)
{
    const IMU_Scaled_t *imu_snap;
    MAGNETOMETER_Calib_t* mag_snap;
    
    imu_update();
    imu_snap = imu_get_fusion_imu();
    s_imu_snap = imu_get_gyro_rate();
    mag_snap = mag_update() ; 
    
    if (imu_snap->timestamp == s_last_imu_time) return;
    s_last_imu_time = imu_snap->timestamp;
    
    float dt_s = imu_snap->dt;
    /* 安全检查 */
    if (dt_s < 0.0002f || dt_s > 0.0005f) {
        dt_s = 0.00025f;
    }

    if (mag_snap->god_time != s_last_mag_time) {
        s_last_mag_time = mag_snap->god_time;
        updateMagBuffer(mag_snap);
    }

    float magNorm = sqrtf(mag_snap->mag_x * mag_snap->mag_x
                        + mag_snap->mag_y * mag_snap->mag_y
                        + mag_snap->mag_z * mag_snap->mag_z);
    uint8_t mag_valid = (magNorm > s_config.mag_threshold_low
                      && magNorm < s_config.mag_threshold_high)
                      && (s_last_mag_time != 0u);

    if (!s_state.is_initialized) {
        if (mag_valid) {
            fusionDynamicInitStep(
                imu_snap->gx, imu_snap->gy, imu_snap->gz,
                imu_snap->ax, imu_snap->ay, imu_snap->az,
                mag_snap->mag_x, mag_snap->mag_y, mag_snap->mag_z
            );
        }
        s_state.timestamp_us = imu_snap->timestamp;

        if (!s_state.is_initialized) {
            return;
        }
    }

    /* 插值获取当前 IMU 时刻的磁力计数据 */
    float mx, my, mz;
    if (interpolateMag(imu_snap->timestamp, &mx, &my, &mz)) {
        MadgwickFusion_UpdateAHRS(
            imu_snap->gx, imu_snap->gy, imu_snap->gz,
            imu_snap->ax, imu_snap->ay, imu_snap->az,
            mx, my, mz,
            dt_s
        );
    } else {
        MadgwickFusion_UpdateIMU(
            imu_snap->gx, imu_snap->gy, imu_snap->gz,
            imu_snap->ax, imu_snap->ay, imu_snap->az,
            dt_s
        );
    }
     const BAROMETER_Real_t* baro_snap = baro_update() ;
    if (baro_snap->god_time != s_last_baro_time) {
        float baro_dt_s = 0.005f;
        if (s_last_baro_time != 0) {
            baro_dt_s = baro_snap->dt;
            if (baro_dt_s < 0.002f || baro_dt_s > 0.05f) {
                baro_dt_s = 0.005f;
            }
        }
        s_last_baro_time = baro_snap->god_time;
        MadgwickFusion_UpdateAltitude((float)baro_snap->press, (float)baro_snap->temp, baro_dt_s);
    }
    
    s_state.timestamp_us = imu_snap->timestamp;
}

