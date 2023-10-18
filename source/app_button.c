/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Device drivers */
#include "fsl_gpio.h"

/* Board */
#include "board.h"
#include "pin_mux.h"

/* Private */
#include "app_button.h"

typedef struct {
    GPIO_Type *gpio;
    uint8_t    port;
    uint8_t    pin;
    bool       active_high;
} app_gpio_t;

static const app_gpio_t s_btn_gpios[] = {
    [APP_BUTTON_CLR] =
        {
            .gpio        = BOARD_INITBTNPINS_BTN_USR_GPIO,
            .port        = BOARD_INITBTNPINS_BTN_USR_PORT,
            .pin         = BOARD_INITBTNPINS_BTN_USR_PIN,
            .active_high = false,
        },
};

int app_button_init(void) {
    /* -- */

    return 0;
}

/**
 * @brief Read button value
 * 
 * @param btn 
 * @return true:  button is pressed
 * @return false: button is not pressed
 */
bool app_button_read(app_button_t btn) {
    const app_gpio_t *btn_pin = &s_btn_gpios[btn];

    uint8_t pin_value = GPIO_PinRead(btn_pin->gpio, btn_pin->port, btn_pin->pin);

    if (btn_pin->active_high) {
        return pin_value ? true : false;
    } else {
        return pin_value ? false : true;
    }
}