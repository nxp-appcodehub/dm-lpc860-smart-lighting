/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef UTIL_FLASH_LPC86X_IAP_H
#define UTIL_FLASH_LPC86X_IAP_H

#include <stdint.h>

/**
 * @brief Note: The slot allocation is done by application.
 *
 */

typedef struct {
    uint8_t start_page; /* The page number in FLASH */
    uint8_t page_count; /* Size, in pages */
} util_flash_t;

int util_flash_init(void);
int util_flash_slot_read(util_flash_t *slot, uint8_t *data);
int util_flash_slot_program(util_flash_t *slot, uint8_t *data);
int util_flash_slot_erase(util_flash_t *slot);
int util_flash_erase_all(void);

#endif