/**
 * @file timestamp.h
 * @brief Free-running 32-bit microsecond counter on TIM2.
 *
 * @details TIM2 is driven directly by this module and is not configured in
 * CubeMX: do not enable it in the .ioc. The counter wraps every ~71 minutes,
 * which the gs_usb host driver handles.
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

#include "main.h"

void timestamp_init(void);

uint32_t timestamp_us(void);

#endif /* TIMESTAMP_H */
