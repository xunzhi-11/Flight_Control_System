#ifndef __FIRST_ORDER_LOWPASS_FILTER_H
#define __FIRST_ORDER_LOWPASS_FILTER_H

#include "data_structure_types.h"
#include "stdbool.h"

typedef struct {
    float alpha_acc ;
    float alpha_gyro ;
    bool is_init ;
    IMU_Scaled_t out ;
} FirstOrderLPF_t ;

void IMU_LPF_Init(FirstOrderLPF_t *lpf, float alpha_acc, float alpha_gyro) ;
void IMU_LPF_Update(FirstOrderLPF_t *lpf, const IMU_Scaled_t *scaled_data) ;

#endif