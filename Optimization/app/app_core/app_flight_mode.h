#ifndef __APP_FLIGHT_MODE_H
#define __APP_FLIGHT_MODE_H

#include "stdbool.h"
#include "stdint.h"

typedef enum {
    FLIGHT_MODE_MANUAL = 0,
    FLIGHT_MODE_ALT_HOLD,
    FLIGHT_MODE_STATIC,   /* ch5=600: outdoor GPS position hold + alt hold */
    FLIGHT_MODE_COUNT
} FlightMode_e;

void FlightMode_Init(void);
FlightMode_e FlightMode_Get(void);
void FlightMode_Set(FlightMode_e mode);

#endif
