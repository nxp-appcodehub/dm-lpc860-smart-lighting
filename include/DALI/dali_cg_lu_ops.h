/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DALI_CG_LU_OPS_H
#define DALI_CG_LU_OPS_H

#include "dali_common.h"

typedef dali_ret_t (*dali_cg_lu_ops_init_t)(void *pdev);
typedef dali_ret_t (*dali_cg_lu_ops_set_level_t)(void *pdev, uint8_t level, uint32_t fade_ms);
typedef dali_ret_t (*dali_cg_lu_ops_get_level_t)(void *pdev, uint8_t *level, bool *is_fading);
typedef dali_ret_t (*dali_cg_lu_ops_load_nvm_t)(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size);
typedef dali_ret_t (*dali_cg_lu_ops_store_nvm_t)(void *pdev, dali_nvm_type_t type, uint8_t *data, uint16_t size);
typedef dali_ret_t (*dali_cg_lu_ops_flush_nvm_t)(void *pdev);

typedef struct {
    dali_cg_lu_ops_init_t      init;
    dali_cg_lu_ops_set_level_t level_set;
    dali_cg_lu_ops_get_level_t level_get;
    dali_cg_lu_ops_load_nvm_t  nvm_load;
    dali_cg_lu_ops_store_nvm_t nvm_store;
    dali_cg_lu_ops_flush_nvm_t nvm_flush;
} dali_cg_lu_ops_t;

#endif 