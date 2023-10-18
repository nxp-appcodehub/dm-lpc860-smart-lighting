/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_PHY_H
#define DALI_PHY_H

#include <stdint.h>

#include "dali_common.h"

typedef enum {
    DALI_PHY_FRAME_24FF    = 24U, /* 24bit Forward Frame */
    DALI_PHY_FRAME_16FF    = 16U, /* 16bit Forward Frame */
    DALI_PHY_FRAME_8BF     = 8U,  /* 8bit Backward Frame */
    DALI_PHY_FRAME_INVALID = 0U,  /* Invalid Frame */
} dali_phy_frame_t;

typedef dali_ret_t (*dali_phy_ops_init_t)(void *pdev);
typedef dali_ret_t (*dali_phy_ops_transmit_t)(void *pdev, dali_phy_frame_t type, uint32_t data);
typedef dali_ret_t (*dali_phy_ops_receive_t)(void *pdev, dali_phy_frame_t *pframe_type, uint32_t *pdata,
                                             uint32_t timeout_ms);

typedef struct {
    dali_phy_ops_init_t     init;
    dali_phy_ops_transmit_t transmit;
    dali_phy_ops_receive_t  receive;
} dali_phy_ops_t;

typedef struct {
    void          *pdev;
    dali_phy_ops_t ops;
} dali_phy_t;

#define DALI_PHY_FRAME_BITS(frame) ((uint8_t)frame) /* Get frame length (in bits) from type */
#define DALI_PHY_FRAME_BITS_MAX    24U

#endif