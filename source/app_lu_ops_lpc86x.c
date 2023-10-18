/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* App */
#include "app_led.h"
#include "util_crc.h"

/* SDK drivers */
#include "fsl_mrt.h"

/* Private header */
#include "app_lu_ops_lpc86x.h"

#define APP_LU_SLOT_SIZE 64
#define APP_LU_NUM_SLOTS 9 /* WARNING: Make sure this is large enough!! */

static bool             s_lu_first_init = true;
static app_lu_handle_t *s_lu_handle_ptr[4];
static uint8_t          s_lu_nvm_cache[APP_LU_NUM_SLOTS][APP_LU_SLOT_SIZE];
static bool             s_lu_nvm_cached[APP_LU_NUM_SLOTS];

/**
 * @brief Initialize Logical Unit operations, will be called on each LU.
 * 
 * @param pdev User parameter
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_init(void *pdev) {
    app_lu_handle_t *ops = pdev;

    /* TODO: This will be called multiple times, workaround for now. */
    if (s_lu_first_init) {
        app_led_init();

        mrt_config_t mrt_cfg = {
            .enableMultiTask = false,
        };

        MRT_Init(MRT0, &mrt_cfg);

        EnableIRQ(MRT0_IRQn);
        NVIC_SetPriority(MRT0_IRQn, 2);

        s_lu_first_init = false;

        for (uint8_t i = 0; i < APP_LU_NUM_SLOTS; i++) {
            s_lu_nvm_cached[i] = false;
        }
    }

    /* Copy pointers here, which will be used in ISR. */
    s_lu_handle_ptr[ops->led_id] = ops;

    ops->current_brightness = 0U;

    MRT_SetupChannelMode(MRT0, ops->led_id, kMRT_RepeatMode);
    MRT_EnableInterrupts(MRT0, ops->led_id, kMRT_TimerInterruptEnable);

    return DALI_RET_OK;
}

/**
 * @brief Set light level and fading, fading is linear within the time specified in fade_ms
 * 
 * @param pdev User parameter
 * @param level target level
 * @param fade_ms fade timeout
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_set_level(void *pdev, uint8_t level, uint32_t fade_ms) {
    app_lu_handle_t *ops = pdev;

    /* Here, we have three conditions.
     * Condition 1: The 31 bit counter is enough for a single step fading (not so slow)
     * Condition 2: The 31 bit counter is not enough for a single step fading (too slow)
     *
     * The maximum allowed step time is 0x7FFFFFFF (2^31 - 1) / Fcore, which will be ~33sec
     * The minimum allowed step time is around 1/3ms, defined in the spec.
     */

    MRT_StopTimer(MRT0, ops->led_id);

    uint8_t steps_delta;

    ops->target_brightness = level;

    if (ops->current_brightness > level) {
        steps_delta = ops->current_brightness - level;
    } else {
        steps_delta = level - ops->current_brightness;
    }

    if ((fade_ms == 0U) || (steps_delta == 0U)) {
        ops->current_brightness = level;
        ops->target_brightness  = level;

        app_led_set_brightness(ops->led_id, level);
    } else {
        uint32_t timer_period = (CLOCK_GetCoreSysClkFreq() / steps_delta) * fade_ms / 1000;
        MRT_StartTimer(MRT0, ops->led_id, timer_period);
    }

    return DALI_RET_OK;
}

/**
 * @brief Get current level and fading status of the Logical Unit, from 0 to 254
 *
 * @param pdev User parameter
 * @param level retrieved level
 * @param is_fading Report the current fading status if this is not NULL
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_get_level(void *pdev, uint8_t *level, bool *is_fading) {
    app_lu_handle_t *ops = pdev;

    *level = app_led_get_brightness(ops->led_id);
    if (*level == 255) {
        *level = 254;
    }

    if (is_fading != NULL) {
        if (MRT_GetStatusFlags(MRT0, ops->led_id) & kMRT_TimerRunFlag) {
            *is_fading = true;
        } else {
            *is_fading = false;
        }
    }

    return DALI_RET_OK;
}

/**
 * @brief Load NVM content from backend store, optional CRC may be provided by user.
 * 
 * @param pdev User parameter
 * @param type NVM type, one of the dali_nvm_type_t enum
 * @param data Loaded data, must be at least `size` byte
 * @param size Size of the data to be loaded.
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_load_nvm(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size) {
    app_lu_handle_t *ops = pdev;

    util_flash_t *nvm_param = NULL;

    switch (type) {
        case DALI_NVM_TYPE_STANDARD:
            nvm_param = &ops->std_nvm_param;
            break;
        case DALI_NVM_TYPE_EXTENDED:
            nvm_param = &ops->ext_nvm_param;
            break;
        case DALI_NVM_TYPE_MEMBANK:
            nvm_param = &ops->membank_param;
            break;
        default:
            break;
    }

    if (nvm_param == NULL) {
        return DALI_RET_FAIL;
    }

    uint8_t  slot_index = nvm_param->start_page;
    uint32_t slot_size  = APP_LU_SLOT_SIZE * nvm_param->page_count;

    if (!s_lu_nvm_cached[slot_index]) {
        if (util_flash_slot_read(nvm_param, s_lu_nvm_cache[slot_index]) != 0) {
            return DALI_RET_FAIL;
        }

        s_lu_nvm_cached[slot_index] = true;
    }

    /* Calculate CRC */
    uint32_t crc = util_crc_calculate(s_lu_nvm_cache[slot_index], slot_size - 4);

    /* Check if CRC is valid */
    if (*(uint32_t *)&s_lu_nvm_cache[slot_index][slot_size - 4] != crc) {
        return DALI_RET_FAIL;
    }

    memcpy(data, s_lu_nvm_cache[slot_index], size);

    return DALI_RET_OK;
}

/**
 * @brief Store the NVM content to backend storage, optional CRC may be provided by user
 * 
 * @param pdev User parameter
 * @param type NVM type, one of the dali_nvm_type_t enum
 * @param data Data to be stored
 * @param size Size of the data to be stored
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_store_nvm(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size) {
    app_lu_handle_t *ops = pdev;

    util_flash_t *nvm_param = NULL;

    switch (type) {
        case DALI_NVM_TYPE_STANDARD:
            nvm_param = &ops->std_nvm_param;
            break;
        case DALI_NVM_TYPE_EXTENDED:
            nvm_param = &ops->ext_nvm_param;
            break;
        case DALI_NVM_TYPE_MEMBANK:
            nvm_param = &ops->membank_param;
            break;
        default:
            break;
    }

    if (nvm_param == NULL) {
        return DALI_RET_FAIL;
    }

    uint8_t  slot_index = nvm_param->start_page;
    uint32_t slot_size  = APP_LU_SLOT_SIZE * nvm_param->page_count;

    memset(s_lu_nvm_cache[slot_index], 0xFF, slot_size);
    memcpy(s_lu_nvm_cache[slot_index], data, size);

    /* Calculate CRC */
    uint32_t crc = util_crc_calculate(s_lu_nvm_cache[nvm_param->start_page], slot_size - 4);

    /* Store CRC to the last 32bit of the slot */
    *(uint32_t *)&s_lu_nvm_cache[slot_index][slot_size - 4] = crc;

    s_lu_nvm_cached[nvm_param->start_page] = true;

    return DALI_RET_OK;
}

/**
 * @brief Flush NVM to backend storage, is used by FLUSH_NVM command
 * 
 * @param pdev User parameter
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t app_lu_ops_flush_nvm(void *pdev) {
    /* Do nothing for now, since we has to keep sector erased for BOD. */
    return DALI_RET_OK;
}

void app_lu_ops_erase_all(void) {
    util_flash_t param = {
        .start_page = 0,
        .page_count = APP_LU_NUM_SLOTS,
    };

    util_flash_slot_erase(&param);
}

/**
 * @brief This is called on BOD ISR, flush everything back to flash.
 * The on-board capacitor might not be enough for all pages, additional bulk caps might be 
 * required (soldered on 3.3V rail) if the flush operation failed (LU NVM lost) on some LUs.
 */
void app_lu_ops_flush_all(void) {
    util_flash_t param = {
        .start_page = 0,
        .page_count = APP_LU_NUM_SLOTS,
    };

    util_flash_slot_program(&param, s_lu_nvm_cache[0]);
}

void MRT0_DriverIRQHandler(void) {
    /* Clear MRT timer flags */
    uint8_t channel_mask = 0U;
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t status = MRT_GetStatusFlags(MRT0, i);
        if (status & kMRT_TimerInterruptFlag) {
            channel_mask |= (1 << i);
            MRT_ClearStatusFlags(MRT0, i, kMRT_TimerInterruptFlag);
        }
    }
    /* Handle brightness change */
    for (uint8_t i = 0; i < 4; i++) {
        if (channel_mask & (1 << i)) {
            app_lu_handle_t *ops = s_lu_handle_ptr[i];

            if (ops->current_brightness < ops->target_brightness) {
                ops->current_brightness++;
                app_led_set_brightness(i, ops->current_brightness);
            } else if (ops->current_brightness > ops->target_brightness) {
                ops->current_brightness--;
                app_led_set_brightness(i, ops->current_brightness);
            } else {
                app_led_set_brightness(i, ops->current_brightness);
                MRT_StopTimer(MRT0, i);
            }
        }
    }
}