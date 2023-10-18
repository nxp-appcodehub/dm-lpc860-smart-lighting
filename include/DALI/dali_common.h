/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_COMMON_H
#define DALI_COMMON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DALI_RET_OK,               /* Operation succeed */
    DALI_RET_FAIL,             /* Something wrong happened */
    DALI_RET_PHY_BUSY_RX,      /* PHY Layer is receiving a frame */
    DALI_RET_PHY_RX_ERROR,     /* PHY Layer reports RX framing error */
    DALI_RET_PHY_RX_TIMEOUT,   /* PHY Layer reports RX timeout */
    DALI_RET_PHY_TX_COLLISION, /* PHY Layer reports a collision during transmission */
    DALI_RET_AC_NO_DEVICE,     /* AC search returns no result */
} dali_ret_t;

typedef enum {
    DALI_UNIT_TYPE_102, /* Control Gears */
    DALI_UNIT_TYPE_103, /* Control Devices */
} dali_unit_type_t;

typedef enum {
    DALI_NVM_TYPE_STANDARD,
    DALI_NVM_TYPE_EXTENDED,
    DALI_NVM_TYPE_MEMBANK,
} dali_nvm_type_t;

#endif