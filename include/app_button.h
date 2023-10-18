/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef APP_BUTTON_H
#define APP_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_BUTTON_CLR = 0U,
} app_button_t;

int app_button_init(void);
bool app_button_read(app_button_t btn);

#endif