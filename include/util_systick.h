/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef UTIL_SYSTICK_H
#define UTIL_SYSTICK_H

#include <stdint.h>

void     util_systick_init(void);
uint32_t util_systick_get(void);
void     util_systick_delay(uint32_t msec);

#endif