#ifndef __JOYCAL_H
#define __JOYCAL_H

#include "stdint.h"
#include "stdbool.h"

typedef struct {
    uint16_t min ;
    uint16_t center ;
    uint16_t max ;
    uint16_t deadband ;
} JoyCalib_Channel_t ;

#define JOY_CALIB_CHANNEL_COUNT   4U

uint16_t Process_Joystick_Channel(uint8_t ch_index, uint16_t raw_adc) ;

void JoyCalib_Get_Default(JoyCalib_Channel_t out[JOY_CALIB_CHANNEL_COUNT]) ;
void JoyCalib_Get_Current(JoyCalib_Channel_t out[JOY_CALIB_CHANNEL_COUNT]) ;
void JoyCalib_Apply(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT]) ;
bool JoyCalib_Is_Valid(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT]) ;

bool JoyCalib_Load(void) ;
bool JoyCalib_Save(const JoyCalib_Channel_t data[JOY_CALIB_CHANNEL_COUNT]) ;

#endif
