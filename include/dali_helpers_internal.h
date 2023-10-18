/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_HELPERS_INTERNAL_H
#define DALI_HELPERS_INTERNAL_H

#include <stdint.h>

#include "dali_cg_lu_ops.h"
#include "dali_phy.h"
#include "dali_sys.h"

typedef void *(*dali_helpers_internal_get_lu_handle_t)(uint8_t lu_id);

void dali_helpers_internal_init(dali_sys_t *psys, dali_phy_t *pphy, const dali_cg_lu_ops_t *ops,
                                dali_helpers_internal_get_lu_handle_t get_handle);
void dali_helpers_internal_process(void);
#endif