#ifndef __APP_MOTOR_UNLOCKER_H
#define __APP_MOTOR_UNLOCKER_H

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"

// fc_safety.h
typedef enum {
    ARM_BLOCK_NONE          = 0,         // 没有任何阻碍
    ARM_BLOCK_CALIBRATING   = (1 << 0),  // IMU还在校准
    ARM_BLOCK_RX_LOSS       = (1 << 1),  // 遥控器未连接或失控
    ARM_BLOCK_THROTTLE_HIGH = (1 << 2),  // 油门没在最低点
    ARM_BLOCK_SWITCH_OFF    = (1 << 3),  // 解锁开关没打下
    ARM_BLOCK_LOW_BATT      = (1 << 4),  // 电池电压太低
    ARM_BLOCK_BARO_CALIBRATING = (1 << 5),  // 气压计零高度标定中
    ARM_BLOCK_MODE_FORBIDDEN = (1 << 6)
} ArmBlockFlag_e;

#define ARM_BLOCK_INIT_STATE (ARM_BLOCK_CALIBRATING | ARM_BLOCK_RX_LOSS | ARM_BLOCK_BARO_CALIBRATING)

void Safety_SetBlock(ArmBlockFlag_e flag) ; 
void Safety_ClearBlock(ArmBlockFlag_e flag) ; 
bool Safety_CanArm(void) ; 
void Safety_RC_FeedDog(void) ; 
bool Safety_RC_Is_Disconnected(void) ; 

#endif

