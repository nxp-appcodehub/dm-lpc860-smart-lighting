/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <stdint.h>

#include "app_adc_shared.h"

#define PH_PD_RESISTOR_VALUE (10000U)
#define PH_LUX_UA 5

/**
 * @brief Get current illuminance (Lux)
 * 
 * @return int Lux
 */
int app_als_read(void) {
    uint32_t adc_voltage = app_adc_shared_get_voltage(APP_ADC_SHARED_CH_ALS);
    uint32_t iph_ua = adc_voltage * 1000 / PH_PD_RESISTOR_VALUE;

    return iph_ua * PH_LUX_UA;
}
