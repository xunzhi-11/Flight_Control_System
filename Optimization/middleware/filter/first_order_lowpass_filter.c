#include "filter/first_order_lowpass_filter.h"
#include "data_structure_types.h"
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"

void IMU_LPF_Init(FirstOrderLPF_t *lpf, float alpha_acc, float alpha_gyro) 
{
    if(lpf == NULL) return ;
    
    // 限制系数在合法范围内 (0.0 ~ 1.0)
    lpf->alpha_acc = (alpha_acc > 1.0f) ? 1.0f : ((alpha_acc < 0.0f) ? 0.0f : alpha_acc) ;
    lpf->alpha_gyro = (alpha_gyro > 1.0f) ? 1.0f : ((alpha_gyro < 0.0f) ? 0.0f : alpha_gyro) ;
    
    lpf->is_init = false ;
}

void IMU_LPF_Update(FirstOrderLPF_t *lpf, const IMU_Scaled_t *scaled_data)
{
    if(lpf == NULL || scaled_data == NULL) return ;

    // 如果是第一次运行，直接将输入对齐到输出，防止从 0 开始的阶跃
    if (!lpf->is_init) {
        lpf->out.ax = (float)scaled_data->ax ;
        lpf->out.ay = (float)scaled_data->ay ;
        lpf->out.az = (float)scaled_data->az ;
        lpf->out.gx = (float)scaled_data->gx ;
        lpf->out.gy = (float)scaled_data->gy ;
        lpf->out.gz = (float)scaled_data->gz ;
        lpf->is_init = true ;
    } else {
        // ====================================================================
        // FPU 硬件乘加 (MAC) 优化算法
        // 原理: Y_new = Y_old + alpha * (X_in - Y_old)
        // 编译器会直接将 "+=" 与右侧的 "*" 映射为 Cortex-M4 的 VMLA 乘加指令
        // ====================================================================
        
        // 加速度计滤波
        lpf->out.ax += lpf->alpha_acc * ((float)scaled_data->ax - lpf->out.ax) ;
        lpf->out.ay += lpf->alpha_acc * ((float)scaled_data->ay - lpf->out.ay) ;
        lpf->out.az += lpf->alpha_acc * ((float)scaled_data->az - lpf->out.az) ;
        
        // 陀螺仪滤波
        lpf->out.gx += lpf->alpha_gyro * ((float)scaled_data->gx - lpf->out.gx) ;
        lpf->out.gy += lpf->alpha_gyro * ((float)scaled_data->gy - lpf->out.gy) ;
        lpf->out.gz += lpf->alpha_gyro * ((float)scaled_data->gz - lpf->out.gz) ;
    }

    // 同步时间戳
    lpf->out.timestamp = scaled_data->timestamp ;
    lpf->out.dt = scaled_data->dt ; 
}