#ifndef __THREE_STAGE_LEVER_H
#define __THREE_STAGE_LEVER_H

typedef enum
{
    THREE_LEVER_MODE_HOVER  = 0 ,
    THREE_LEVER_MODE_STATIC = 1 ,
    THREE_LEVER_MODE_MANUAL = 2 ,
} three_lever_mode_e ;

three_lever_mode_e get_three_lever_mode(void) ;

#endif
