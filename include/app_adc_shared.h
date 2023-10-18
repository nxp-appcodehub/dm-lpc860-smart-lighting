/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef APP_ADC_SHARED_H
#define APP_ADC_SHARED_H

typedef enum {
    APP_ADC_SHARED_CH_ALS   = 8U,  /* ADC_8 */
    APP_ADC_SHARED_CH_NTC   = 9U,  /* ADC_9 */
    APP_ADC_SHARED_CH_POT   = 10U, /* ADC_10 */
    APP_ADC_SHARED_CH_LED_R = 2U,  /* ADC_2 */
    APP_ADC_SHARED_CH_LED_G = 1U,  /* ADC_1 */
    APP_ADC_SHARED_CH_LED_B = 3U,  /* ADC_3 */
} app_adc_shared_channel_t;

typedef void (*app_adc_cb_t)(void);

int  app_adc_shared_init(void);
int  app_adc_shared_start(void);
int  app_adc_shared_stop(void);
int  app_adc_shared_deinit(void);
int  app_adc_shared_get_result(app_adc_shared_channel_t ch);
int  app_adc_shared_get_voltage(app_adc_shared_channel_t ch);
void app_adc_shared_set_callback(app_adc_cb_t cb);

#endif /* APP_ADC_SHARED_H */