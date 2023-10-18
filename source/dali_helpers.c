/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <stddef.h>

/* Helpers */
#include "app_lu_ops_lpc86x.h"
#include "dali_cg_lu_ops.h"
#include "dali_phy_ftm.h"
#include "dali_sys_bm.h"
#include "util_flash_lpc86x_iap.h"
#include "util_systick.h"

/* App */
#include "app_adc_shared.h"
#include "app_als.h"
#include "app_bod.h"
#include "app_button.h"
#include "app_led.h"
#include "app_ntc.h"
#include "app_pot.h"

/* Internal helpers */
#include "dali_helpers_internal.h"

#define DEMO_LU_COUNT 3

static dali_phy_t s_phy = {
    .ops =
        {
            .init     = dali_phy_ftm_init,
            .transmit = dali_phy_ftm_transmit,
            .receive  = dali_phy_ftm_receive,
        },
};

static dali_sys_t s_sys = {
    .ops =
        {
            .init   = dali_sys_bm_init,
            .delay  = dali_sys_bm_delay,
            .mstick = dali_sys_bm_mstick,
            .random = dali_sys_bm_random,
        },
};

static const dali_cg_lu_ops_t s_lu_ops = {
    .init      = app_lu_ops_init,
    .level_set = app_lu_ops_set_level,
    .level_get = app_lu_ops_get_level,
    .nvm_load  = app_lu_ops_load_nvm,
    .nvm_store = app_lu_ops_store_nvm,
    .nvm_flush = app_lu_ops_flush_nvm,
};

/* Store the parameters of NVM pages. */
static app_lu_handle_t s_lu_handle[DEMO_LU_COUNT] = {
    {
        .led_id        = APP_LED_CH_R,
        .std_nvm_param = {.start_page = 0x00, .page_count = 1}, /* Offset  0x00 */
        .ext_nvm_param = {.start_page = 0x01, .page_count = 1}, /* Offset  0x40 */
        .membank_param = {.start_page = 0x02, .page_count = 1}, /* Offset  0x80 */
    },
    {
        .led_id        = APP_LED_CH_G,
        .std_nvm_param = {.start_page = 0x03, .page_count = 1}, /* Offset  0xC0 */
        .ext_nvm_param = {.start_page = 0x04, .page_count = 1}, /* Offset 0x100 */
        .membank_param = {.start_page = 0x05, .page_count = 1}, /* Offset 0x140 */
    },
    {
        .led_id        = APP_LED_CH_B,
        .std_nvm_param = {.start_page = 0x06, .page_count = 1}, /* Offset 0x180 */
        .ext_nvm_param = {.start_page = 0x07, .page_count = 1}, /* Offset 0x1C0 */
        .membank_param = {.start_page = 0x08, .page_count = 1}, /* Offset 0x200 */
    },
};

static void *dali_helper_get_lu_handle(uint8_t lu_id) {
    return &s_lu_handle[lu_id];
}

void dali_helper_init(void) {
    app_button_init();
    app_bod_init();
    app_adc_shared_init();
    app_adc_shared_start();
    app_led_init();

    // Press USER button while powering the module will erase non-volatile configurations
    if (app_button_read(APP_BUTTON_CLR)) {
        app_lu_ops_erase_all();

        for (uint8_t i = 0; i < 3; i++) {
            app_led_set_brightness(APP_LED_CH_B, 20);
            util_systick_delay(1000);
            app_led_set_brightness(APP_LED_CH_B, 0);
            util_systick_delay(1000);
        }
    }

    dali_helpers_internal_init(&s_sys, &s_phy, &s_lu_ops, dali_helper_get_lu_handle);

    // Erase flash to prepare for the BOD
    app_lu_ops_erase_all();
}

void dali_helper_task(void) {
    dali_helpers_internal_process();
}