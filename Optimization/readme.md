1. 本工程为了优化性能做了很多不算规范的行为和不规范写法，在移植时需要考虑自己的设计是否兼容以下设计，以免硬件行为异常

- bmp390l
a. EXTI9_5_IRQHandler 默认是EXTI_LINE_5 发来的，省去判断开销
b. EXTI9_5_IRQHandler中不调用LL_EXTI_ClearFlag_0_31，而是直接写寄存器EXTI->PR来清除标志位
c. 不判断回调是否挂载，默认初始化完成挂载
d. TIM5_GetStamp 直接返回寄存器数值
e. IIC_Dev_DMA_TC_Handler里直接调用设备回调而不判断是否为空
f. 对博世官方的温度压强补偿算法用秦九韶算法优化，计算结果不变
g. 整个bmp390l一次drdy消耗共计12000左右条指令（开启了单精度fpu计算），截止到提供真实气压和温度数据

- bmm150
a. EXTI1_IRQHandler中不调用LL_EXTI_ClearFlag_0_31，而是直接写寄存器EXTI->PR来清除标志位
b. 整个bmm150一次drdy消耗共计700左右条指令（开启了单精度fpu计算）,截止到提供trim补偿后的数据

- icm42688
a. EXTI0_IRQHandler中不调用LL_EXTI_ClearFlag_0_31，而是直接写寄存器EXTI->PR来清除标志位
b. 整个icm42688一次drdy消耗共计300左右条指令（开启了单精度fpu计算），截止到提供原始数据
c.将原始数据交付给madgwick共计消耗100左右条指令

2. 实际性能开销
