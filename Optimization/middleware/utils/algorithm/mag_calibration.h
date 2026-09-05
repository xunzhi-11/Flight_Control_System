#ifndef __MAG_CALIBRATION_H
#define __MAG_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

// 统一的校准参数句柄
typedef struct {
    float hard_iron_offset[3];   // 3x1 向量
    float soft_iron_matrix[3][3]; // 3x3 矩阵
} MagCal_Param_t;

/**
 * @brief 离线校准计算（输入采集到的原始数据，输出校准矩阵）
 * @param data 采集到的三轴数据数组 (注意：此函数内部会修改 data 的值以进行去均值化)
 * @param num_samples 数据点的数量
 * @param out_param 计算输出的校准参数
 * @return true 成功收敛, false 数据无效或不收敛
 */
bool MagCal_ComputeParams(float data[][3], uint16_t num_samples, MagCal_Param_t *out_param);

/**
 * @brief 在线实时应用（在中断或任务中，对单个数据点进行实时校准）
 * @param raw_in 传感器原始读取值 (纯物理量，单位微特斯拉)
 * @param param 之前算好的校准参数
 * @param cal_out 校准后的干净数据
 */
void MagCal_Apply(const float raw_in[3], const MagCal_Param_t *param, float cal_out[3]);

#endif