#include "app_core/app_motor_unlocker.h"
#include "tim/tim5.h"
#include "stdbool.h"
#include "stdint.h"

#define SAFETY_JUDGE_LOSS_TIME        200000U

volatile uint32_t arming_disable_flags = ARM_BLOCK_INIT_STATE;
volatile uint64_t last_rc_frame_time = 0 ; 


void Safety_SetBlock(ArmBlockFlag_e flag) 
{
    arming_disable_flags |= flag;
}

void Safety_ClearBlock(ArmBlockFlag_e flag) 
{
    arming_disable_flags &= ~flag;
}

bool Safety_CanArm(void) 
{
    return (arming_disable_flags == ARM_BLOCK_NONE); 
}

void Safety_RC_FeedDog(void)
{
    last_rc_frame_time = TIM5_GetStamp() ; 
}

bool Safety_RC_Is_Disconnected(void) 
{
    // 获取当前时间
    uint32_t current_time = TIM5_GetStamp();

    if ((current_time - last_rc_frame_time) > SAFETY_JUDGE_LOSS_TIME) 
    {
        Safety_SetBlock(ARM_BLOCK_RX_LOSS) ; 
        return true; 
    }
    Safety_ClearBlock(ARM_BLOCK_RX_LOSS) ; 
    return false;
}