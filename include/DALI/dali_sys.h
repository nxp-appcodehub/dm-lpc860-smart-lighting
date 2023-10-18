/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_SYS_H
#define DALI_SYS_H

#include "dali_common.h"

typedef dali_ret_t (*dali_sys_ops_init_t)(void *pdev);
typedef dali_ret_t (*dali_sys_ops_delay_t)(void *pdev, uint32_t msec);
typedef dali_ret_t (*dali_sys_ops_mstick_t)(void *pdev, uint32_t *msec);
typedef dali_ret_t (*dali_sys_ops_random_t)(void *pdev, uint32_t *random);

typedef struct {
    dali_sys_ops_init_t   init;
    dali_sys_ops_delay_t  delay;
    dali_sys_ops_mstick_t mstick;
    dali_sys_ops_random_t random;
} dali_sys_ops_t;

typedef struct {
    dali_sys_ops_t ops;
    void          *pdev;
} dali_sys_t;

#endif