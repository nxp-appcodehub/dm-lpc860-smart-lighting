/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <stdlib.h>

#include "fsl_iap.h"

/* Private header */
#include "dali_sys_bm.h"
#include "util_systick.h"

/**
 * @brief Initialize system utilities
 * 
 * @param pdev Pointer to user parameter
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t dali_sys_bm_init(void *pdev) {
    /* Future may requires initialization. */
    uint32_t uid[4];

    /* Seed the random sequence using device unique ID. */
    IAP_ReadUniqueID(uid);

    uint32_t final_seed = uid[0] + uid[1] + uid[2] + uid[3];
    srand((int)final_seed);

    return DALI_RET_OK;
}

/**
 * @brief Delay for specified milliseconds
 * 
 * @param pdev Pointer to user parameter
 * @param msec milliseconds to delay
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t dali_sys_bm_delay(void *pdev, uint32_t msec) {
    /* pdev currently unused */
    util_systick_delay(msec);

    return DALI_RET_OK;
}

/**
 * @brief Generate a 32-bit random number
 * 
 * @param pdev Pointer to user parameter 
 * @param random Pointer to the random number
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t dali_sys_bm_random(void *pdev, uint32_t *random) {
    *random = rand();

    return DALI_RET_OK;
}

/**
 * @brief Returns current tick in milliseconds since boot
 * 
 * @param pdev Pointer to user parameter  
 * @param msec Pointer to the tick count
 * @return dali_ret_t DALI_RET_OK for success, others for errors.
 */
dali_ret_t dali_sys_bm_mstick(void *pdev, uint32_t *msec) {
    *msec = util_systick_get();

    return DALI_RET_OK;
}