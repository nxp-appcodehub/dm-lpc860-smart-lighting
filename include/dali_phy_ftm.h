/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_PHY_FTM_BM_H
#define DALI_PHY_FTM_BM_H

#include "dali_phy.h"

dali_ret_t dali_phy_ftm_init(void *pdev);
dali_ret_t dali_phy_ftm_transmit(void *pdev, dali_phy_frame_t frame_type, uint32_t frame);
dali_ret_t dali_phy_ftm_receive(void *pdev, dali_phy_frame_t *pframe_type, uint32_t *pframe, uint32_t timeout_ms);

#endif