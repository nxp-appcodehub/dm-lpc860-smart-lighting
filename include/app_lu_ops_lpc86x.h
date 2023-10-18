/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_CG_LU_OPS_LPC84X_H
#define DALI_CG_LU_OPS_LPC84X_H

#include <stdbool.h>
#include <stdint.h>

#include "dali_common.h"

/* Utilities */
#include "util_flash_lpc86x_iap.h"

typedef struct {
    uint8_t      led_id;
    uint8_t      target_brightness;  /* Fade target */
    uint8_t      current_brightness; /* Current value */
    util_flash_t std_nvm_param;
    util_flash_t ext_nvm_param;
    util_flash_t membank_param;
} app_lu_handle_t;

dali_ret_t app_lu_ops_init(void *pdev);
dali_ret_t app_lu_ops_set_level(void *pdev, uint8_t level, uint32_t fade_ms);
dali_ret_t app_lu_ops_get_level(void *pdev, uint8_t *level, bool *is_fading);
dali_ret_t app_lu_ops_load_nvm(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size);
dali_ret_t app_lu_ops_store_nvm(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size);
dali_ret_t app_lu_ops_flush_nvm(void *pdev);

void app_lu_ops_erase_all(void);
void app_lu_ops_flush_all(void);

#endif