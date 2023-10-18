/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_SYS_BM_H
#define DALI_SYS_BM_H

#include "dali_sys.h"

dali_ret_t dali_sys_bm_init(void *pdev);
dali_ret_t dali_sys_bm_delay(void *pdev, uint32_t msec);
dali_ret_t dali_sys_bm_random(void *pdev, uint32_t *msec);
dali_ret_t dali_sys_bm_mstick(void *pdev, uint32_t *msec);

#endif