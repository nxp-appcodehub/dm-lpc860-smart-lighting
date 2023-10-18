/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* SDK board */
#include "board.h"
#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"

/* Debug console */
#include "fsl_debug_console.h"

/* Helpers */
#include "dali_phy_ftm.h"
#include "dali_sys_bm.h"
#include "util_flash_lpc86x_iap.h"
#include "util_systick.h"

void dali_helper_init(void);
void dali_helper_task(void);

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    BOARD_InitDebugConsole();

    PRINTF("Hello world.\r\n");

    util_flash_init();
    util_systick_init();

    dali_helper_init();

    for (;;) {
        dali_helper_task();
    }
}