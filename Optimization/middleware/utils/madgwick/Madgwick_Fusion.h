/**
 * @file    madgwick_fusion.h
 * @brief   Madgwick AHRS 姿态融合算法
 * @details 融合 ICM42688(IMU) + BMM150(磁力计) + BMP390L(气压计)
 *
 * 坐标系约定（全链路统一，无 NWU 中间层）：
 *   - 地球系 NED：X 北、Y 东、Z 地
 *   - 机体系 FRD：X 前、Y 右、Z 下（与 IMU/磁力计 PCB 安装一致）
 *   - 水平静止时：roll ≈ 0°，pitch ≈ 0°，az ≈ +1g
 *   - 欧拉角：Roll 绕 X，Pitch 绕 Y，Yaw 绕 Z（右手系，度）
 */

#ifndef __MADGWICK_FUSION_H
#define __MADGWICK_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "data_structure_types.h"

/*============================================================================*/
/*                              宏定义                                         */
/*============================================================================*/

#define MADGWICK_DEFAULT_BETA           0.1f    /* 默认滤波器增益 */
#define MADGWICK_DEFAULT_ZETA           0.0f    /* 陀螺仪漂移补偿增益 */

/*============================================================================*/
/*                              数据类型定义                                    */
/*============================================================================*/

/**
 * @brief 四元数结构体
 */
typedef struct {
    float w;    /* q0 - 标量部分 */
    float x;    /* q1 - 矢量 i 分量 */
    float y;    /* q2 - 矢量 j 分量 */
    float z;    /* q3 - 矢量 k 分量 */
} Quaternion_t;

/**
 * @brief 欧拉角结构体 (单位：度)
 */
typedef struct {
    float roll;     /* 横滚角 (-180 ~ 180) */
    float pitch;    /* 俯仰角 (-90 ~ 90)   */
    float yaw;      /* 偏航角 (-180 ~ 180) */
} EulerAngle_t;


/**
 * @brief 角速度结构体 (单位：度\秒)
 */
typedef struct {
    float roll_rate;     /* 横滚角速度 */
    float pitch_rate;    /* 俯仰角速度 */
    float yaw_rate;      /* 偏航角速度 */
} AngularVelocity_t;

/**
 * @brief 高度估计结构体
 */
typedef struct {
    float altitude;             /* 相对高度 (m) */
    float vertical_velocity;    /* 垂直速度 (m/s) */
    float sea_level_pressure;   /* 海平面参考气压 (Pa) */
    float temperature;          /* 温度 (℃) */
} AltitudeState_t;

/**
 * @brief 陀螺仪偏差估计
 */
typedef struct {
    float x;    /* x轴偏差 (rad/s) */
    float y;    /* y轴偏差 (rad/s) */
    float z;    /* z轴偏差 (rad/s) */
} GyroBias_t;

/**
 * @brief 融合状态输出结构体
 */
typedef struct {
    /* 四元数 */
    Quaternion_t    quaternion;
    
    /* 欧拉角 */
    EulerAngle_t    euler;
    
    /* 高度状态 */
    AltitudeState_t altitude_state;
    
    /* 陀螺仪偏差估计 */
    GyroBias_t      gyro_bias;
    
    AngularVelocity_t     angular_velocity ; 

    /* 线性加速度 (去除重力后, 单位: g) */
    float linear_acc_x;
    float linear_acc_y;
    float linear_acc_z;
    
    /* 时间信息 */
    uint32_t    timestamp_us;       /* 主机时间戳 (us) */
    float       dt;                 /* 本次更新间隔 (s) */
    uint32_t    update_count;       /* 更新计数 */
    
    /* 状态标志 */
    uint8_t     is_initialized;     /* 初始化完成标志 */
    uint8_t     mag_valid;          /* 磁力计数据有效标志 */
    uint8_t     baro_valid;         /* 气压计数据有效标志 */
} MadgwickFusion_State_t;

/**
 * @brief 融合算法配置参数
 */
typedef struct {
    float beta;
    float zeta;
    float mag_declination;
    float acc_threshold_low;
    float acc_threshold_high;
    /* 新增磁力计阈值 */
    float mag_threshold_low;
    float mag_threshold_high;
    float altitude_filter_alpha;    /* 气压高度低通系数 (baro 修正前) */
    float baro_alt_correct_gain;    /* 气压高度修正增益 */
    float baro_vel_correct_gain;    /* 气压速度修正增益 */
} MadgwickFusion_Config_t;

/*============================================================================*/
/*                              API 函数声明                                   */
/*============================================================================*/

/**
 * @brief  初始化 Madgwick 融合算法
 * @param  config: 配置参数指针, 为 NULL 时使用默认参数
 * @retval 无
 */
void MadgwickFusion_Init(const MadgwickFusion_Config_t *config);

/**
 * @brief  重置融合状态
 * @retval 无
 */
void MadgwickFusion_Reset(void);

/**
 * @brief  主更新函数 - 自动读取传感器数据并更新姿态
 * @note   建议在 IMU 数据就绪中断中调用, 频率 1kHz
 * @retval 无
 */
void MadgwickFusion_Update(void);

void MadgwickFusion_Update_Nomag(void) ; 

/**
 * @brief  仅使用 IMU 更新 (6轴, 无磁力计)
 * @param  gx, gy, gz: 角速度 (°/s)
 * @param  ax, ay, az: 加速度 (g)
 * @param  dt: 时间间隔 (s)
 * @retval 无
 */
void MadgwickFusion_UpdateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt);

/**
 * @brief  使用 AHRS 更新 (9轴, 含磁力计)
 * @param  gx, gy, gz: 角速度 (°/s)
 * @param  ax, ay, az: 加速度 (g)
 * @param  mx, my, mz: 磁场强度 (uT)
 * @param  dt: 时间间隔 (s)
 * @retval 无
 */
void MadgwickFusion_UpdateAHRS(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz,
                                float dt);

/**
 * @brief  气压计修正高度估计 (IMU 积分在 AHRS 更新中完成)
 * @param  pressure: 气压值 (Pa)
 * @param  temperature: 温度 (℃)
 * @param  dt: 气压采样间隔 (s)
 * @retval 无
 */
void MadgwickFusion_UpdateAltitude(float pressure, float temperature, float dt);

/**
 * @brief  获取融合状态指针 (只读)
 * @retval 状态结构体常量指针
 */
const MadgwickFusion_State_t* MadgwickFusion_GetStatePtr(void);

/**
 * @brief  获取本周期 Madgwick 更新时使用的 IMU 快照（只读）
 * @note   须在 MadgwickFusion_Update() 之后、同一控制周期内使用
 */
const IMU_Scaled_t* MadgwickFusion_GetImuSnap(void);

/**
 * @brief  设置海平面参考气压 (用于高度计算)
 * @param  pressure: 海平面气压 (Pa), 默认 101325 Pa
 * @retval 无
 */
void MadgwickFusion_SetSeaLevelPressure(float pressure);

/**
 * @brief  用标定时刻气压建立零高度（offset 取瞬时气压高度，不用慢滤波）
 */
void MadgwickFusion_CalibrateAltitudeAtPressure(float pressure_pa, float temperature_c);

/**
 * @brief  校准当前位置为零高度（使用内部滤波后的气压高度）
 * @retval 无
 */
void MadgwickFusion_CalibrateAltitude(void);

/**
 * @brief  设置滤波器增益 Beta
 * @param  beta: 增益值 (0.0 ~ 1.0, 推荐 0.033 ~ 0.5)
 * @retval 无
 */
void MadgwickFusion_SetBeta(float beta);

void MadgwickFusion_SetZeta(float zeta) ; 

/**
 * @brief  设置磁偏角
 * @param  declination: 磁偏角 (度)
 * @retval 无
 */
void MadgwickFusion_SetMagDeclination(float declination);

/**
 * @brief  获取旋转矩阵 (3x3)
 * @param  matrix: 输出矩阵 [9] (行优先)
 * @retval 无
 */
void MadgwickFusion_GetRotationMatrix(float matrix[9]);

/**
 * @brief  将机体坐标系向量转换到地球坐标系
 * @param  bx, by, bz: 机体坐标系向量
 * @param  ex, ey, ez: 地球坐标系向量输出
 * @retval 无
 */
void MadgwickFusion_BodyToEarth(float bx, float by, float bz,
                                 float *ex, float *ey, float *ez);

/**
 * @brief  将地球坐标系向量转换到机体坐标系
 * @param  ex, ey, ez: 地球坐标系向量
 * @param  bx, by, bz: 机体坐标系向量输出
 * @retval 无
 */
void MadgwickFusion_EarthToBody(float ex, float ey, float ez,
                                 float *bx, float *by, float *bz);


/**
 * @brief  带插值的主更新函数
 * @note   对磁力计数据进行线性插值，提高时间对齐精度
 *         适用于对航向角精度要求较高的场景
 * @retval 无
 */
void MadgwickFusion_UpdateWithInterpolation(void);




#ifdef __cplusplus
}
#endif

#endif /* __MADGWICK_FUSION_H */