/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef APP_LED_H
#define APP_LED_H

#include <stdint.h>

typedef enum {
    APP_LED_CH_B = 0U,
    APP_LED_CH_R = 1U,
    APP_LED_CH_G = 2U,
} app_led_channel_t;

int     app_led_init(void);
int     app_led_deinit(void);
void    app_led_set_brightness(app_led_channel_t led_ch, uint8_t curr_ma);
uint8_t app_led_get_brightness(app_led_channel_t led_ch);

#endif