#ifndef __5_WAY_KEY_H
#define __5_WAY_KEY_H

#include "stdint.h"

typedef enum
{
    ACTION_NONE = 0 , 
    ACTION_UP = 1 , 
    ACTION_DOWN = 2 , 
    ACTION_ENTER = 3 , 
    ACTION_BACK = 4 , 
    ACTION_RETURN = 5
} Input_Action_e ;

typedef void (*Key_Event_Cb_t)(Input_Action_e action) ;

void Key_RegisterCallback(Key_Event_Cb_t cb) ;
void Key_Scan(void) ; 

#endif
