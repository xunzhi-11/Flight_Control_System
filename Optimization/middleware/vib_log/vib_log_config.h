#ifndef VIB_LOG_CONFIG_H
#define VIB_LOG_CONFIG_H

#include "stdint.h"

/* Master switch: set 0 to strip all VIBLOG code at compile time. */
#ifndef VIBLOG_ENABLE
#define VIBLOG_ENABLE 0
#endif

#if VIBLOG_ENABLE

/* 4 kHz IMU feed -> decim 4 ~= 1 kHz wire (matches UART sustained throughput). */
#ifndef VIBLOG_DECIM_RATIO
#define VIBLOG_DECIM_RATIO 4U
#endif

/* Gyro int16 scale: deg/s * 100 (must match tools/vib_protocol.py GYRO_SCALE). */
#ifndef VIBLOG_GYRO_SCALE
#define VIBLOG_GYRO_SCALE 100.0f
#endif

/* Select default sink registered in VibLog_Init(). */
#ifndef VIBLOG_PORT_USART1
#define VIBLOG_PORT_USART1 1
#endif

/* Auto-enable streaming after VibLog_Init(). */
#ifndef VIBLOG_AUTO_ENABLE
#define VIBLOG_AUTO_ENABLE 1
#endif

#endif /* VIBLOG_ENABLE */

#endif /* VIB_LOG_CONFIG_H */
