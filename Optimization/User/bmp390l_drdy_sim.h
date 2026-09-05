#ifndef BMP390L_DRDY_SIM_H
#define BMP390L_DRDY_SIM_H

/*
 * 临时方案：BMP390L INT 损坏时，用 PC1 模拟 DRDY，短接 PC1 -> PB5 触发 EXTI。
 * 新传感器到货后删除 User/bmp390l_drdy_sim.c、本头文件，并移除 System_Application.c 中的引用。
 */
void bmp390l_drdy_sim_hw_init(void);
void bmp390l_drdy_sim_task_start(void);

#endif
