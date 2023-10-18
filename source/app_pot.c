/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "app_adc_shared.h"

#define ADC_RANGE_FULL 4096

/**
 * @brief Get potentiometer readings
 * 
 * @return int percentage, in 0.1%
 */
int app_pot_read(void) {
    return app_adc_shared_get_result(APP_ADC_SHARED_CH_POT) * 1000 / ADC_RANGE_FULL;
}
