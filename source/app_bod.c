/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Board */
#include "pin_mux.h"

/* SDK drivers */
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_syscon.h"

/* Private */
#include "app_adc_shared.h"
#include "app_led.h"
#include "app_lu_ops_lpc86x.h"

#define BOD_IRQ_LEVEL kBod_InterruptLevel3

int app_bod_init(void) {
    POWER_DisablePD(kPDRUNCFG_PD_BOD);

    SDK_DelayAtLeastUs(30, CLOCK_GetCoreSysClkFreq());

    NVIC_ClearPendingIRQ(BOD_IRQn);

    POWER_SetBodLevel(kBod_ResetLevelReserved, BOD_IRQ_LEVEL, false);

    EnableIRQ(BOD_IRQn);

    return 0;
}

void BOD_DriverIRQHandler(void) {
    GPIO_PinWrite(BOARD_INITDBGPINS_DBG2_GPIO, BOARD_INITDBGPINS_DBG2_PORT, BOARD_INITDBGPINS_DBG2_PIN, 1);
    app_lu_ops_flush_all();
    GPIO_PinWrite(BOARD_INITDBGPINS_DBG2_GPIO, BOARD_INITDBGPINS_DBG2_PORT, BOARD_INITDBGPINS_DBG2_PIN, 0);

    for(;;) {
        /* - Do not leave this function since we are BOD'd - */
    }
}
