/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef UTIL_CRC_H
#define UTIL_CRC_H

#include <stdint.h>

void     util_crc_init(void);
uint32_t util_crc_calculate(uint8_t *data, uint16_t size);

#endif