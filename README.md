一套飞控系统

硬件配置：
飞控：
icm42688,bmm150,bmp390l,stm32f407vet6,elrs_2.4g_rx,atgm336h
f450机架，1-45桨叶，920kv2212电机
3s 2700mah lipo

遥控器：
双轴霍尔摇杆*2，五向按键，jumpert-2.4g-tx-pro，oled
自制pcb

软件配置：
飞控端集成madgwick、notch filter、aff filter、dshot600、crsf parser、crc8、safe_lock等数学控制工具
采用drdy -> exti -> isr -> dma_trigger_cb -> dma_tc_isr -> data_handler_cb的数据链路处理传感器数据
采用单核模拟双核的形式，以freertos_it_max_priority为界限，将硬实时任务放isr完成，软实时任务放freertos_task完成
解算频率4khz，调控频率1khz.
